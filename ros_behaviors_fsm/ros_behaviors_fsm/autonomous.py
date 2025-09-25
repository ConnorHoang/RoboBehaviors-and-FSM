""" This node uses the laser scan measurement pointing straight ahead from
    the robot and compares it to a desired set distance.  The forward velocity
    of the robot is adjusted until the robot achieves the desired distance """

from time import sleep
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from neato2_interfaces.msg import Bump
from threading import Thread, Event
import numpy as np

# Default angle of sector to check lidar data, in rad
DEFAULT_ANGLE_SWEEP = 15 * math.pi / 180

ANGLE_RIGHT_START = 75 * math.pi / 180
ANGLE_RIGHT_END = 105 * math.pi / 180

ANGLE_FRONT_START = 155 * math.pi / 180
ANGLE_FRONT_END = 205 * math.pi / 180

WALL_MIN_PTS = 5

WALL_SEARCH_TOLERANCE = 0.15

class NeatoFsm(Node):
    """ 
    ROS2 node performing autonomous wall identification / approach /
    following functionality as a finite state machine. 

    Attributes:
        use_teleop: whether robot should yield to teleop control
        vel_pub: ROS2 publisher to /cmd_vel topic
        wall_vis_pub: publisher of MarkerArray defining wall shape
        wall_line_pub: publisher of Marker defining perceived wall
        target_line_pub: publisher of Marker defining wall follow
            path to drive on 
        Kp_drive: Coefficient for proportional control on wall approach
        target_distance: distance at which to follow a wall (meters)
        identify_wall_distance: distance at which the robot recognizes
            a wall and attempts to follow it (meters)
        max_vel: maximum linear velocity of robot (m/s)
        min_vel: minimum nonzero linear velocity of robot (m/s)
        max_ang_vel: maximum angular velocity of robot (rad/s)
        angle_correction
        velocity: Twist object to pass to drive() commands
        bumped: Event to set when bump sensor is pressed
        state: autonomous state of the robot
            Can be one of 'approach','wall_search','wall_follow',
            'turn', or 'bump'
        main_loop_thread: Thread running main execution loop
        scan_msg: last recorded LIDAR scan
        correction_angle: current angle to correct wall follow
        integral: integral term for PID in wall follow
        pid_controls: list of kP, kI, and kD coeffs for PID
        last_loop_time: time iteration of last run loop  
    """
    def __init__(self):
        super().__init__('neato_fsm') # Call ROS Node initializer 

        # Create loop timer + subscribers to ROS topics
        self.create_timer(0.1, self.run_loop)
        # subscriber to see if we should listen to teleop
        self.create_subscription(Bool,'use_teleop',self.teleop_callback,qos_profile=qos_profile_sensor_data)
        # Subscriber to intake laser sensor data
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        # Subscriber to bump sensor data
        self.create_subscription(Bump,'bump',self.process_bump, qos_profile=qos_profile_sensor_data)

        # Create ROS topic publishers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.wall_vis_pub = self.create_publisher(MarkerArray,'wall_marker', 10)
        self.wall_line_pub = self.create_publisher(Marker,'wall_line', 10)
        self.target_line_pub = self.create_publisher(Marker,'target_line', 10)

        # Declare ROS2 params
        self.declare_parameter("Kp_drive",0.5)
        self.Kp_drive = self.get_parameter("Kp_drive").get_parameter_value().double_value
        self.declare_parameter("target_distance",0.5)
        self.target_distance = self.get_parameter("target_distance").get_parameter_value().double_value
        self.declare_parameter("identify_wall_distance",1.0)
        self.identify_wall_distance = self.get_parameter("identify_wall_distance").get_parameter_value().double_value
        self.declare_parameter("max_vel",0.2)
        self.max_vel = self.get_parameter("max_vel").get_parameter_value().double_value
        self.declare_parameter("min_vel",0.08)
        self.min_vel = self.get_parameter("min_vel").get_parameter_value().double_value
        self.declare_parameter("max_ang_vel",0.5)
        self.max_ang_vel = self.get_parameter("max_ang_vel").get_parameter_value().double_value
        self.declare_parameter("pid_controls",[0.5,0.0,1.0])
        self.pid_controls = self.get_parameter("pid_controls").get_parameter_value().double_array_value

        # Parameter callback updates instance attributes when params
        # are changed via ROS
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Instantiate instance attributes
        self.use_teleop = True
        self.velocity = Twist()
        self.bumped = Event()
        self.state = "wall_search"
        self.main_loop_thread = Thread(target=self.run_loop)
        self.scan_msg = None
        self.correction_angle = 0.0
        self.integral = 0 
        self.last_loop_time = self.get_clock().now()
        

    def parameters_callback(self, params):
        """
        Update corresponding class attributes every time a ROS2 
        parameter is updated. 

        Args:
            params (list[rclpy.parameter.Parameter]): list of ROS2
            params updated.

        Returns:
            rcl_interfaces.masg.SetParameterResult with successful=True,
            indicating parameters were updated successfully. 
        """
        for param in params:
            if param.name == "Kp_drive":
                self.Kp_drive = param.value
            elif param.name == "target_distance":
                self.target_distance = param.value
            elif param.name == "max_vel":
                self.max_vel = param.value
            elif param.name == "min_vel":
                self.min_vel = param.value
            elif param.name == "max_ang_vel":
                self.max_ang_vel = param.value
            elif param.name == "angle_correction":
                self.angle_correction = param.value
            elif param.name == "angle_correction_tolerance":
                self.angle_correction_tolerance = param.value
            elif param.name == "identify_wall_distance":
                self.identify_wall_distance = param.value
            elif param.name == "pid_controls":
                self.pid_controls = param.value
        return SetParametersResult(successful=True)

    def run_loop(self):
        """
        Primary run loop governing the robot state machine.
        If the robot is set to listen to teleop commands rather 
        than drive autonomously, it does not execute the logic.
        Otherwise, calculates time interval since last loop, scans
        LIDAR data to check to the front and right, and executes state
        logic based on a switch statement. 

        State changes are dependent on wall presence in front of the
        robot, to the right of the robot, and the bump sensor state. 
        """
        if self.use_teleop:
            self.state = "wall_search"
        else:
            try:
                current_time = self.get_clock().now()
                self.dt = (current_time - self.last_loop_time).nanoseconds / 1e9
                self.last_loop_time = current_time

                # Wait for the first LIDAR scan data before acting
                if self.scan_msg:
                    dist_right,_,dist_right_list,angle_right_list = self.get_scan_angle(ANGLE_RIGHT_START, ANGLE_RIGHT_END)
                    dist_front,_,dist_front_list,angle_front_list = self.get_scan_angle(ANGLE_FRONT_START, ANGLE_FRONT_END)
                    
                    print(f"state: {self.state}")
                    print(f"dist front: {dist_front}, dist right: {dist_right}\n")

                    # Define and publish array of Markers defining walls to front and sides
                    marker_arr = []
                    for i in range(len(dist_front_list)):
                        marker_arr.append(self.marker_from_lidar(dist_front_list[i],angle_front_list[i],i))
                    for i in range(len(dist_right_list)):
                        marker_arr.append(self.marker_from_lidar(dist_right_list[i],angle_right_list[i],i+len(dist_front_list)))
                    arr_to_publish = MarkerArray()
                    arr_to_publish.markers = marker_arr
                    self.wall_vis_pub.publish(arr_to_publish)

                    # Logic dependent on current FSM state
                    match self.state:
                        case "approach":
                            # Approach velocity is proportional to distance
                            # from target,
                            # constrained between min_vel and max_vel
                            approach_vel = self.Kp_drive * (dist_front - self.target_distance)
                            approach_vel = min(self.max_vel,max(self.min_vel,approach_vel))
                            self.drive(self.velocity, linear=approach_vel, angular=0.0)
                            
                            #print(f"linear_vel: {approach_vel}")
                            sleep(0.1)
                        
                            # State change: turn if facing wall                        
                            if dist_front < self.target_distance:
                                self.state = "turn"

                        case "wall_follow":
                            # Robot can detect a wall to the side
                            # We should follow and make velocity 
                            # adjustments to stay target distance from 
                            # wall. If further than identify_wall_distance,
                            # we've lost our wall --- go back to approach

                            self.wall_follow()

                            # Facing wall, turn away from it 
                            if dist_front < self.target_distance:
                                self.state = "turn"

                            # Too far from wall, go back to approach
                            if dist_right >= self.identify_wall_distance:
                                self.state = "approach"

                        case "turn":
                            # Robot can detect a wall in front 
                            # We should turn until the path ahead is clear
                            self.turn(dist_front)

                            # If clear path ahead
                            if dist_front >= self.target_distance:
                                # Follow wall if wall to right
                                if dist_right < self.identify_wall_distance:
                                    self.state = "wall_follow"
                                    self.previous_correction_angle = 0.0
                                    self.previous_error = 0.0
                                else:
                                    # Otherwise, approach wall ahead
                                    self.state = "approach"

                        case "wall_search":
                            # Robot cannot detect a wall in front of it
                            # Find the nearest wall and turn until
                            # facing a wall at nearest distance

                            closest_dist_list,_,_ = self.closest_wall_dist()

                            self.turn()
                            print(f"closest dist: {closest_dist_list}, dist front: {dist_front}, diff: {abs(closest_dist_list[0] - dist_front)}")
                            
                            # If facing a wall as close as the closest
                            # wall, start approaching it
                            if abs(closest_dist_list[0] - dist_front) < WALL_SEARCH_TOLERANCE:
                                self.state = "approach"

                        case "bump":
                            # Bump sensor has been depressed
                            # We should stop moving immediately
                            # Can be exited via teleop control
                            self.drive(self.velocity,linear=0.0,angular=0.0)
                        case _:
                            # Undefined state, throw an error
                            raise(ValueError(f"State {self.state} is not defined")) 
                        
                    self.vel_pub.publish(self.velocity)
            except KeyboardInterrupt:
                self.drive(self.velocity,linear=0.0,angular=0.0)

    def polar_to_cartesian(self,r_list,theta_list):
        """
        Maps polar coordinates from LIDAR to (x,y) pairs in the same
        refernce frame. 

        Args:
            r_list (list[float]): list of distances measured by LIDAR
            theta_list (list[float]): angles in radians for LIDAR scans

        Returns:
            x_list (list[float]): X coords of corresponding points
            y_list (list[float]): Y coords of corresponding points
        """
        x_list = [r_list[i] * math.cos(theta_list[i]) for i in range(len(r_list))]
        y_list = [r_list[i] * math.sin(theta_list[i]) for i in range(len(r_list))]
        #print(f"xlist: {x_list}, ylist: {y_list}")
        return x_list,y_list

    def marker_from_lidar(self,dist,angle,id):
        """
        Defines a marker to visualize wall presence in Rviz2
        Marker position is specified in angular coords in
        frame of robot LIDAR scanner

        Args:
            dist (float): radial distance from origin of marker
            angle (float): angle (rad) from origin of marker
            id (int): unique ID to identify marker

        Returns:
            visualization_msgs.msg.Marker object at the specified
            position
        """

        # Instantiate marker in the laser frame at proper pos
        wall_marker = Marker()
        wall_marker.header.frame_id = "base_laser_link"
        wall_marker.header.stamp = self.get_clock().now().to_msg()
        wall_marker.pose.position.x = math.cos(angle) * dist
        wall_marker.pose.position.y = math.sin(angle) * dist
        wall_marker.pose.position.z = 0.0
        wall_marker.pose.orientation.z = math.tan(angle)

        # Visual appearance of marker
        wall_marker.color.r = 1.0
        wall_marker.color.g = 0.0
        wall_marker.color.b = 0.0
        wall_marker.color.a = 1.0
        wall_marker.scale.x = 0.3
        wall_marker.scale.y = 0.1
        wall_marker.scale.z = 0.1

        # Markers require unique IDs to all get displayed
        wall_marker.id = id
        return wall_marker


    def process_scan(self, msg):
        """
        Callback to save LIDAR scan to class attribute
        
        Args:
            msg (sensor_msgs.msg.LaserScan): msg to write
            to self.scan_msg attribute
        """
        self.scan_msg = msg

    def teleop_callback(self, msg):
        """
        Records boolean data declaring if teleop mode is in use
        
        Args:
            msg (std_msgs.msg.Bool): Bool message, true if robot
            should yield to teleop control
        """
        self.use_teleop = msg.data

    def wall_identify(self, x_list,y_list):
        """
        Approximate a wall shape as linear given points on it

        For the given (x,y) points, identifies the principal
        components where pc1 points along the wall direction, pc2
        is normal to the wall, and the mean is a point on the wall. 

        Calculates target path as an offset from the wall, difference
        in linear position between robot and path, and difference in
        heading.

        Args:
            x_list (list[float]): X coords to fit data to
            y_list (list[float]): Y coords to fit data to

        Returns:
            err_linear (float): difference between robot position and
            target line position
            err_angular (float) difference in angle (radians) between 
            robot heading and target line heading
            wall_line_marker (visualization_msgs.msg.Marker): Marker 
            indicating perceived wall position to visualize
            wall_target_marker (visualization_msgs.msg.Marker): Marker 
            indicating indented path to follow
        """
        data = np.column_stack([x_list, y_list])

        # do pca using numpy svd rather than scikit learn
        mean = data.mean(axis=0)
        Q = data - mean
        _, _, Vt = np.linalg.svd(Q, full_matrices=False)
        t = Vt[0]                                   
        pc1 = t / np.linalg.norm(t)
        pc2 = np.array([-t[1], t[0]]) # pc2 is normal to pc1 

        # normal should face the origin 
        if np.dot(mean,pc2) > 0.0:
            pc2 *= -1

        mean_target = mean + self.target_distance * pc2

        wall_line_marker = self.make_line_marker(mean,pc1,True)
        wall_target_marker = self.make_line_marker(mean_target,pc1,False)

        err_linear = np.dot(mean_target,pc2) # linear distance of neato from wall
        angle_wall = np.atan2(pc1[1],pc1[0])
        err_angular = angle_wall

        #print(f"mean: {mean}, pc1: {pc1}, pc2: {pc2}")

        return err_linear,err_angular,wall_line_marker,wall_target_marker

    def make_line_marker(self,start,tangent,is_wall):
        """
        Defines a marker depicting a line, to use for visualizing
        wall identification

        Args:
            start (np.ndarray): x,y pos to start line segment on
            normal (np.ndarray): x,y vector describing unit tangent
            to wall
            is_wall (bool): True if line should be colored red (wall),
            False if line should be colored green (path)
        
        Returns:
            visualization_msgs.msg.Marker representing wall or path
        """

        # Define start pt and length of line
        length = 3
        start_pt = Point()
        start_pt.x = start[0]
        start_pt.y = start[1]
        start_pt.z = 0.0

        # Get endpt by adding scaled unit tangent to start
        end_pt = Point()
        end_pt.x = start[0]+length*tangent[0]
        end_pt.y = start[1]+length*tangent[1]
        end_pt.z = 0.0

        # Define marker start, end, frame, and stamp
        wall_marker = Marker()
        wall_marker.header.frame_id = "base_laser_link"
        wall_marker.header.stamp = self.get_clock().now().to_msg()
        wall_marker.points.append(start_pt)
        wall_marker.points.append(end_pt)
        wall_marker.type = 4 # Line segment type == 4 

        # Color red for wall, green for path
        if is_wall:
            wall_marker.color.r = 1.0
            wall_marker.color.g = 0.0
        else:
            wall_marker.color.r = 0.0
            wall_marker.color.g = 1.0
        wall_marker.color.b = 0.0
        wall_marker.color.a = 1.0
        wall_marker.scale.x = 0.1
        wall_marker.id = 0
        return wall_marker

    def get_scan_angle(self,min_angle=DEFAULT_ANGLE_SWEEP*-0.5, max_angle=DEFAULT_ANGLE_SWEEP*0.5):
        """
        Finds the minimum radial distance to an obstacle between the
        specified angles. Used for state change logic dependent on
        boolean checks of if a wall is close to the front or right

        Args:
            min_angle (float): Angle to begin measurement at
            max_angle (float): Angle to end measurement at

        Returns:
            min_dist (float): radial distance to closest point
            angle_min_dist (float): angle of closest point
            dist_arr: (list[float]): array of distances of points
                within angle bounds
            angle_arr: (list[float]): array of angles of points
                within angle bounds
        """
        min_robot_angle = self.scan_msg.angle_min
        max_robot_angle = self.scan_msg.angle_max
        increment = self.scan_msg.angle_increment

        min_dist = None
        angle_min_dist = None
        dist_arr = []
        angle_arr = []
        for index,range in enumerate(self.scan_msg.ranges):
            ray_angle = ((min_robot_angle + index*increment) % 360)
            #print(f"checking range {range} at index {index}, angle = {min_robot_angle + index*increment}, min robot angle = {min_robot_angle}, inc = {increment}")
            if (min_angle < ray_angle and max_angle > ray_angle):
                dist_arr.append(range)
                angle_arr.append(ray_angle)
                if (not min_dist or range < min_dist):
                    min_dist = range
                    angle_min_dist = ray_angle
        return min_dist,angle_min_dist,dist_arr,angle_arr

    def drive(self, msgArg, linear, angular):
        """
        Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """   
        msg = msgArg
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def process_bump(self, msg):
        """
        Callback for handling a bump sensor input."
        Input: 
            msg (Bump): a Bump type message from the subscriber.
        """
        # Set the bump Event if any part of the sensor is pressed
        if (msg.left_front == 1 or \
                           msg.right_front == 1 or \
                           msg.left_side == 1 or \
                           msg.right_side == 1):
            self.bumped.set()
            self.state = "bump"
            self.drive(self.velocity,linear=0.0,angular=0.0)   

    def turn(self, ccw=True):
        """
        Turn command, called in turn and wall_search states. 
        Turns at maximum set angular velocity.
        Direction of turn specified by ccw argument
        
        Args:
            ccw (bool): Whether to turn counterclockwise
        """

        if ccw:
            self.drive(self.velocity, linear=0.0, angular=self.max_ang_vel)
        else:
            self.drive(self.velocity, linear=0.0, angular=-1*self.max_ang_vel)
        sleep(0.1)

    def closest_wall_dist(self,angle_start=None,angle_end=None):
        """
        Search last recorded scan data for the nearest wall. 
        To prvent sensor noise from registering an obstacle closer than there
        is one, the maximum distance across WALL_MIN_PTS measurements is used
        Return lists of distances/angles of points on identified wall

        Args:
            angle_start (float): angle to start wall sweep in
            angle_end (float): angle to end wall sweep in

        Returns:
            range_list (list[float]): a list of radial distances to
                the points on the nearest identified wall
            ray_angle_list (list[float]): a list of the corresponding
                angles defining those points on the wall
            ccw (bool): direction to turn to face the identified wall
        """
        min_wall_dist = None
        min_wall_dist_index = None
        ray_angle_list = []

        for index,dist in enumerate(self.scan_msg.ranges):
            ray_angle = ((self.scan_msg.angle_min + index*self.scan_msg.angle_increment) % 360)
            
            # If bounds are defined, ensure ray_angle is within bounds
            if (angle_end is not None) and (angle_start is not None):
                if ray_angle < angle_start or ray_angle > angle_end:
                    continue
            if index >= WALL_MIN_PTS:
                # Dist is the maximum distance of each window of 
                # WALL_MIN_PTS points, to deal with noise
                wall_dist = max(self.scan_msg.ranges[index-WALL_MIN_PTS:index])
                if not min_wall_dist or wall_dist < min_wall_dist:
                    min_wall_dist = wall_dist
                    min_wall_dist_index = index
        
        # Identify quickest rotation direction to reach angle
        if min_wall_dist_index > (len(self.scan_msg.ranges)*0.5):
            ccw = False
        else:
            ccw = True

        for i in range(int(min_wall_dist_index-WALL_MIN_PTS),int(min_wall_dist_index)):
            # add corresponding ray angles to ray_angle_list after identifying wall
            ray_angle_list.append(((self.scan_msg.angle_min + i*self.scan_msg.angle_increment) % 360))

        return self.scan_msg.ranges[min_wall_dist_index-WALL_MIN_PTS:min_wall_dist_index],ray_angle_list,ccw

    def wall_follow(self):
        """
        Drives parallel to wall. Identifies target path offset from
        wall, calculates error between position and target path, and
        uses error, accumulated error over time, and change in error
        to correct heading towards the target path. 

        Correction behavior can be tuned via the pid_controls param
        """

        if not hasattr(self, 'previous_error'):
            self.previous_error = 0.0

        if not hasattr(self, 'previous_correction_angle'):
            self.previous_correction_angle = 0.0

        ### Currently just try to identify a wall on the right side
        dist_list,angle_list,_ = self.closest_wall_dist(angle_start=ANGLE_RIGHT_START,angle_end=ANGLE_RIGHT_END)

        # Calculate error, publish visualizations on wall and path
        x_list,y_list = self.polar_to_cartesian(dist_list,angle_list)
        err_linear,err_angular,marker_wall,marker_target = self.wall_identify(x_list,y_list)
        print(f"linear error: {err_linear}")
        self.wall_line_pub.publish(marker_wall)
        self.target_line_pub.publish(marker_target)

        # pass error through PID controller to get angle adjustment
        control, self.previous_error, self.integral = self.pid_controller(0, -1*err_linear, self.pid_controls[0], self.pid_controls[1], self.pid_controls[2], self.previous_error, self.integral, self.dt)
        
        if self.dt > 0:
            angular_acceleration = (self.correction_angle - self.previous_correction_angle) / self.dt
        
        # apply correction to hopefully reduce error
        self.correction_angle = max(min(self.max_ang_vel,control),-1*self.max_ang_vel)

        print(f"error: {self.previous_error}, control: {control}, correction_angle: {self.correction_angle}, angular_acceleration: {angular_acceleration},  integral: {self.integral}")
        
        self.drive(self.velocity, self.max_vel, angular=self.correction_angle)

    def pid_controller(self, setpoint, pv, kp, ki, kd, previous_error, integral, dt):
        """
        Parametrized PID controller implementation
        returns control terms, calculated error, and integral term
        Used to control angular velocity for wall following

        Args:
            setpoint: target state for controller
            pv: process variable (current state)
            kp: proportional coefficient
            ki: integral coefficient
            kd: derivative coefficient
            previous_error: error in last iter, used by derivative
            integral: accumulated error over time
            dt: timestep since last iteration

        Returns: 
            control: value to use for correction
            error: calculated error, setpoint - pv
            integral: integral constrained to bounds
        """
        error = setpoint - pv

        integral += error * dt
        max_integral = 1.0
        integral = max(-max_integral, min(max_integral, integral)) # windup protection

        if dt > 0.0:
            derivative = (error - previous_error) / dt
        else: 
            derivative = 0.0

        control = kp * error + ki * integral + kd * derivative
        return control, error, integral

    def reset_pid_state(self):
        """Resets PID controller"""
        self.previous_error = 0.0
        self.integral = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = NeatoFsm()
    rclpy.spin(node)
    node.drive(node.velocity,linear=0.0,angular=0.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
