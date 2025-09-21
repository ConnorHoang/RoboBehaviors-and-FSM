""" This node uses the laser scan measurement pointing straight ahead from
    the robot and compares it to a desired set distance.  The forward velocity
    of the robot is adjusted until the robot achieves the desired distance """

from time import sleep
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from neato2_interfaces.msg import Bump
from threading import Thread, Event

# Default angle of sector to check lidar data, in rad
DEFAULT_ANGLE_SWEEP = 15 * math.pi / 180

ANGLE_RIGHT_START = 75 * math.pi / 180
ANGLE_RIGHT_END = 105 * math.pi / 180

ANGLE_FRONT_START = 155 * math.pi / 180
ANGLE_FRONT_END = 205 * math.pi / 180

WALL_MIN_PTS = 5

WALL_SEARCH_TOLERANCE = 0.15

class NeatoFsm(Node):
    """ This class wraps the basic functionality of the node """
    def __init__(self):
        super().__init__('neato_fsm')
        """Combine all below comments to actually decent docstring""" ##
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        # Subscriber to intake laser sensor data
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        # Subscriber to bump sensor data
        #self.create_subscription(Bump,'bump',self.process_bump, qos_profile=qos_profile_sensor_data)
        # publisher to send Neato velocity to ROS space
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.distance_to_obstacle = None
        # Kp is the constant or to apply to the proportional error signal
        self.declare_parameter("Kp_drive",0.5)
        self.Kp_drive = self.get_parameter("Kp_drive").get_parameter_value().double_value
        # target_distance is the desired distance to the obstacle in front
        self.declare_parameter("target_distance",1.0)
        self.target_distance = self.get_parameter("target_distance").get_parameter_value().double_value
        # maximum allowable linear speed of Neato
        self.declare_parameter("max_vel",0.2)
        self.max_vel = self.get_parameter("max_vel").get_parameter_value().double_value
        # minimum allowable linear speed of Neato during approach
        self.declare_parameter("min_vel",0.08)
        self.min_vel = self.get_parameter("min_vel").get_parameter_value().double_value
        # Max allowable angular velocity
        self.declare_parameter("max_ang_vel",0.5)
        self.max_ang_vel = self.get_parameter("max_ang_vel").get_parameter_value().double_value
        # Angular velocity correction to add/subtract when following wall
        self.declare_parameter("angle_correction",0.1)
        self.angle_correction = self.get_parameter("angle_correction").get_parameter_value().double_value
        # Tolerance for drifting further/closer to wall when following
        self.declare_parameter("angle_correction_tolerance",0.01)
        self.angle_correction_tolerance = self.get_parameter("angle_correction_tolerance").get_parameter_value().double_value
        # velocity (speed and angle) of Neato
        self.velocity = Twist()
        # bump boolean for physical sensor where True is bumped
        self.bumped = Event()
        # FSM state the robot is currently in
        self.state = "approach" ### Change to be "wall_search"
        # array of last 5 distances to right wall
        self.right_dist_list = [0.0, 0.0, 0.0, 0.0, 0.0]
        # Thread to process main loop logic
        self.main_loop_thread = Thread(target=self.run_loop)
        # Last recorded LIDAR scan
        self.scan_msg = None
        # Angular velocity for adjusting wall distance
        self.correction_angle = 0.0
        # integral for PID
        self.integral = 0

    def parameters_callback(self, params):
        """Callback for whenever a parameter is changed."""
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
        return SetParametersResult(successful=True)

    def run_loop(self):
        """Primary loop"""
        try:
            self.drive(self.velocity, linear=0.0, angular=0.0)
            self.add_on_set_parameters_callback(self.parameters_callback)

            # Wait for the first LIDAR scan data before acting
            if self.scan_msg:
                dist_right = self.get_scan_angle(ANGLE_RIGHT_START, ANGLE_RIGHT_END)
                dist_front = self.get_scan_angle(ANGLE_FRONT_START, ANGLE_FRONT_END)
                self.right_dist_list.pop(0)
                self.right_dist_list.append(dist_right)
                print(f"state: {self.state}")
                print(f"dist front: {dist_front}, dist right: {dist_right}\n")
                match self.state:
                    case "approach":
                        # Approach velocity is proportional to distance from target dist from wall
                        # Constrained between min_vel and max_vel
                        approach_vel = self.Kp_drive * (dist_front - self.target_distance)
                        approach_vel = min(self.max_vel,max(self.min_vel,approach_vel))
                        self.drive(self.velocity, linear=approach_vel, angular=0.0)
                        print(f"linear_vel: {approach_vel}")
                        sleep(0.1)
                    
                        if dist_front < self.target_distance:
                            self.state = "turn"

                    case "wall_follow":
                        # Robot can detect a wall to the side
                        # We should follow and make velocity adjustments to stay
                        # target distance from wall
                        self.wall_follow(dist_front,dist_right,self.right_dist_list,self.target_distance)
                        if dist_front < self.target_distance:
                            self.state = "turn"

                        if dist_right >= self.target_distance:
                            self.state = "approach"

                    case "turn":
                        # Robot can detect a wall in front (positive x direction)
                        # We should turn until the path ahead is clear
                        self.turn(dist_front)

                        if dist_front >= self.target_distance:
                            if dist_right < self.target_distance:
                                self.state = "wall_follow"
                            else:
                                self.state = "approach"

                    case "wall_search":
                        # Robot cannot detect a wall in front of it
                        # We query LIDAR data to find the direction of the nearest wall
                        # If our 

                        closest_dist,ccw = self.closest_wall_dist()

                        self.turn(ccw=ccw)
                        print(f"closest dist: {closest_dist}, dist front: {dist_front}, diff: {abs(closest_dist - dist_front)}")
                        #sleep(0.1)
                        if abs(closest_dist - dist_front) < WALL_SEARCH_TOLERANCE:
                            self.state = "approach"

                    case "bump":
                        # Bump sensor has been depressed
                        # We should stop moving immediately
                        self.drive(self.velocity,linear=0.0,angular=0.0)
                    case _:
                        # Undefined state, throw an error
                        raise(ValueError(f"State {self.state} is not defined")) 
                    
                self.vel_pub.publish(self.velocity)
        except KeyboardInterrupt:
            self.drive(self.velocity,linear=0.0,angular=0.0)

    def process_scan(self, msg):
        """Check if distance from min to max angle from neato neato is less then target distance"""
        self.scan_msg = msg

    def get_scan_angle(self,min_angle=DEFAULT_ANGLE_SWEEP*-0.5, max_angle=DEFAULT_ANGLE_SWEEP*0.5):
        """
        Queries the most recent LIDAR scan data for nearest wall distance across a given swept angle
        """
        min_robot_angle = self.scan_msg.angle_min
        max_robot_angle = self.scan_msg.angle_max
        increment = self.scan_msg.angle_increment

        min_dist = None
        for index,range in enumerate(self.scan_msg.ranges):
            #print(f"checking range {range} at index {index}, angle = {min_robot_angle + index*increment}, min robot angle = {min_robot_angle}, inc = {increment}")
            if (min_angle < ((min_robot_angle + index*increment) % 360) and max_angle > ((min_robot_angle + index*increment) % 360)):
                if (not min_dist or range < min_dist):
                    min_dist = range
        return min_dist

    def drive(self, msgArg, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """   
        msg = msgArg
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def process_bump(self, msg):
        """Callback for handling a bump sensor input."
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
        """Turn state. Turn counterclockwise until Neato is ~parallel to wall on right and no wall in front
        
        Args:
            msg (Twist()): 
        Action:
            Turn counterclockwise
        """
        ccw = True
        if ccw:
            self.drive(self.velocity, linear=0.0, angular=self.max_ang_vel)
        else:
            self.drive(self.velocity, linear=0.0, angular=-1*self.max_ang_vel)
        sleep(0.1)

    def closest_wall_dist(self):
        """
        Search last recorded scan data for the nearest wall. 
        To prvent sensor noise from registering an obstacle closer than there
        is one, the maximum distance across WALL_MIN_PTS measurements is used
        """
        min_wall_dist = None
        min_wall_dist_index = None

        for index,range in enumerate(self.scan_msg.ranges):
            if index >= WALL_MIN_PTS:
                wall_dist = max(self.scan_msg.ranges[index-WALL_MIN_PTS:index])
                if not min_wall_dist or wall_dist < min_wall_dist:
                    min_wall_dist = wall_dist
                    min_wall_dist_index = index
        
        if min_wall_dist_index > (len(self.scan_msg.ranges)*0.5):
            ccw = False
        else:
            ccw = True

        return self.scan_msg.ranges[min_wall_dist_index],ccw

    def wall_follow(self, dist_front, dist_right, right_dist_list, target_dist):
        """
        Drives in parralel to wall. Self-correcting by detecing distance wall on the right, comparing to average of previous times, 
        and turning to stay within range of wall. Tuning is important

        Args:
            
        Action;
            Move foward
            Turn right/left
        """

        previous_error = [x - target_dist for x in right_dist_list]

        print(f"integral: {self.integral}")
        control, error, self.integral = self.pid_controller(target_dist, dist_right, 0.1, 0, 0, previous_error, self.integral, 1)

        self.correction_angle = max(min(self.max_ang_vel,control),-1*self.max_ang_vel)
        self.drive(self.velocity, self.max_vel, angular=self.correction_angle)

    def pid_controller(self, setpoint, pv, kp, ki, kd, previous_error, integral, dt):
        error = setpoint - pv
        integral += error * dt
        derivative = (error - max(previous_error)) / dt # remove max and fix later
        control = kp * error + ki * integral + kd * derivative
        return control, error, integral

def main(args=None):
    rclpy.init(args=args)
    node = NeatoFsm()
    rclpy.spin(node)
    node.drive(node.velocity,linear=0.0,angular=0.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()





# retain info when transitioning between states to modify state behavior?


""" 

state: wall_approach
- drive fowrward
t: at wall -> move to turn

state: wall_follow
- drive forward
- if decreasing in distance to wall, angle out of wall
- if increasing in distance to wall, andle into wall

state: turn
- drive forward
- turn away from wall

"""
