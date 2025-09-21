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

ANGLE_RIGHT_START = 80 * math.pi / 180
ANGLE_RIGHT_END = 100 * math.pi / 180

ANGLE_FRONT_START = 170 * math.pi / 180
ANGLE_FRONT_END = 190 * math.pi / 180

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
        self.create_subscription(Bump,'bump',self.process_bump, qos_profile=qos_profile_sensor_data)
        # publisher to send Neato velocity to ROS space
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.distance_to_obstacle = None
        # Kp is the constant or to apply to the proportional error signal
        self.declare_parameter("Kp",0.5)
        # target_distance is the desired distance to the obstacle in front
        self.declare_parameter("target_distance",1.0)
        # value to trigger stop in driving
        self.stop = False
        # maximum allowable linear speed of Neato
        self.declare_parameter("max_vel",0.2)
        # minimum allowable linear speed of Neato during approach
        self.declare_parameter("min_vel",0.08)
        # Max allowable angular velocity
        self.declare_parameter("max_ang_vel",0.5)
        # Angular velocity correction to add/subtract when following wall
        self.declare_parameter("angle_correction",0.1)
        # Tolerance for drifting further/closer to wall when following
        self.declare_parameter("angle_correction_tolerance",0.01)
        # velocity (speed and angle) of Neato
        self.velocity = Twist()
        # bump boolean for physical sensor where True is bumped
        self.bumped = Event()
        # FSM state the robot is currently in
        self.state = "approach"
        # array of last 5 distances to right wall
        self.err_list = [0.0, 0.0, 0.0, 0.0, 0.0]
        # Thread to process main loop logic
        self.main_loop_thread = Thread(target=self.run_loop)
        # Last recorded LIDAR scan
        self.scan_msg = None
        # Angular velocity for adjusting wall distance
        self.correction_angle = 0.0
        # integral for PID
        self.integral = 0

    def run_loop(self):
        """Primary loop"""
        self.drive(self.velocity, linear=0.0, angular=0.0)

        # Wait for the first LIDAR scan data before acting
        if self.scan_msg:
            dist_right = self.get_scan_angle(ANGLE_RIGHT_START, ANGLE_RIGHT_END)
            dist_front = self.get_scan_angle(ANGLE_FRONT_START, ANGLE_FRONT_END)
            self.err_list.pop(0)
            self.err_list.append(dist_right)
            print(f"state: {self.state}")
            print(f"dist front: {dist_front}, dist right: {dist_right}\n")
            match self.state:
                case "approach":
                    if not self.stop:
                        # Approach velocity is proportional to distance from target dist from wall
                        # Constrained between min_vel and max_vel
                        approach_vel = self.get_parameter("Kp").get_parameter_value().double_value * (dist_front - self.get_parameter("target_distance").get_parameter_value().double_value)
                        approach_vel = min(self.get_parameter("max_vel").get_parameter_value().double_value,max(self.get_parameter("min_vel").get_parameter_value().double_value,approach_vel))
                        self.drive(self.velocity, linear=approach_vel, angular=0.0)
                        sleep(0.1)
                    
                    if dist_front < self.get_parameter("target_distance").get_parameter_value().double_value:
                        self.state = "turn"

                case "wall_follow":
                    # Robot can detect a wall to the side
                    # We should follow and make velocity adjustments to stay
                    # target distance from wall
                    dist_right_avg = sum(self.err_list)/max(1,len(self.err_list))
                    self.wall_follow(dist_front,dist_right,dist_right_avg,self.get_parameter("target_distance").get_parameter_value().double_value)
                    if dist_front < self.get_parameter("target_distance").get_parameter_value().double_value:
                        self.state = "turn"

                    if dist_right >= self.get_parameter("target_distance").get_parameter_value().double_value:
                        self.state = "approach"

                case "turn":
                    # Robot can detect a wall in front (positive x direction)
                    # We should turn until the path ahead is clear
                    self.turn(dist_front)

                    if dist_front >= self.get_parameter("target_distance").get_parameter_value().double_value:
                        if dist_right < self.get_parameter("target_distance").get_parameter_value().double_value:
                            self.state = "wall_follow"
                        else:
                            self.state = "approach"

                case "bump":
                    # Bump sensor has been depressed
                    # We should stop moving immediately
                    self.drive(self.velocity,linear=0.0,angular=0.0)
                case _:
                    # Undefined state, throw an error
                    raise(ValueError(f"State {self.state} is not defined")) 
                
            self.vel_pub.publish(self.velocity)

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
            print(f"checking range {range} at index {index}, angle = {min_robot_angle + index*increment}, min robot angle = {min_robot_angle}, inc = {increment}")
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

    def turn(self, dist_front):
        """Turn state. Turn counterclockwise until Neato is ~parallel to wall on right and no wall in front
        
        Args:
            msg (Twist()): 
        Action:
            Turn counterclockwise
        """

        self.drive(self.velocity, linear=0.0, angular=self.get_parameter("max_ang_vel").get_parameter_value().double_value)
        sleep(0.1)

    def wall_follow(self, dist_front, dist_right, dist_right_avg, target_dist):
        """
        Drives in parralel to wall. Self-correcting by detecing distance wall on the right, comparing to average of previous times, 
        and turning to stay within range of wall. Tuning is important

        Args:
            
        Action;
            Move foward
            Turn right/left
        """

        previous_error = dist_right_avg - target_dist

        print(f"integral: {self.integral}")
        control, error, self.integral = self.pid_controller(setpoint=target_dist, pv=dist_right, kp=0.1, ki=0, kd=0, previous_error=previous_error, integral=self.integral, dt=1)

        self.correction_angle = max(min(self.get_parameter("max_ang_vel").get_parameter_value().double_value,control),-1*self.get_parameter("max_ang_vel").get_parameter_value().double_value)
        self.drive(self.velocity, self.get_parameter("max_vel").get_parameter_value().double_value, angular=self.correction_angle)

    def pid_controller(self, setpoint, pv, kp, ki, kd, previous_error, integral, dt):
        error = setpoint - pv
        integral += error * dt
        derivative = (error - previous_error) / dt
        control = kp * error + ki * integral + kd * derivative
        return control, error, integral

def main(args=None):
    rclpy.init(args=args)
    node = NeatoFsm()
    rclpy.spin(node)
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