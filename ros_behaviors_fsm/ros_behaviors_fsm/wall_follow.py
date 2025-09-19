""" This node uses the laser scan measurement pointing straight ahead from
    the robot and compares it to a desired set distance.  The forward velocity
    of the robot is adjusted until the robot achieves the desired distance """

from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from neato2_interfaces.msg import Bump
from threading import Thread, Event

# Default angle of sector to check lidar data, in rad
DEFAULT_ANGLE_SWEEP = 0.5

class NeatoFsm(Node):
    """ This class wraps the basic functionality of the node """
    def __init__(self):
        super().__init__('neato_fsm')
        """Combine all below comments to actually decent docstring""" ##
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        # Subscriber to intake laser sensor data
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        # publisher to send Neato velocity to ROS space
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.distance_to_obstacle = None
        # Kp is the constant or to apply to the proportional error signal
        self.Kp = 0.4
        # target_distance is the desired distance to the obstacle in front
        self.target_distance = 1.2
        # value to trigger stop in driving
        self.stop = False
        # value for forward velocity of neato
        self.vel = 0.3
        # velocity (speed and angle) of Neato
        self.velocity = Twist()
        # bump boolean for physical sensor where True is bumped
        self.bumped = Event()
        # FSM state the robot is currently in
        self.state = "approach"
        # Thread to process main loop logic
        self.main_loop_thread = Thread(target=self.run_loop)

    def run_loop(self):
        """Primary loop"""
        self.drive(linear=0.0, angular=0.0)

        if not self.stop:
            self.drive(self.velocity, linear=0.5, angular=0.0)
            sleep(0.1)

        match self.state:
            case "approach":
                # Robot doesn't know the current location of a wall
                # By default we drive forward until we get close enough
                # to a wall
                pass
            case "wall_follow":
                # Robot can detect a wall to the side
                # We should follow and make velocity adjustments to stay
                # target distance from wall
                pass
            case "wall_turn":
                # Robot can detect a wall in front (positive x direction)
                # We should turn until the path ahead is clear
                pass
            case "bump":
                # Bump sensor has been depressed
                # We should stop moving immediately
                self.drive(self.velocity,linear=[0,0,0],angular=[0,0,0])
            case _:
                # Undefined state, throw an error
                raise(ValueError(f"State {self.state} is not defined")) 

    def process_scan(self, msg):
        """Check if distance from min to max angle from neato neato is less then target distance"""
        self.scan_msg = msg

    def get_scan_angle(self,min_angle=DEFAULT_ANGLE_SWEEP*-0.5, max_angle=DEFAULT_ANGLE_SWEEP*0.5):
        """
        Queries the most recent LIDAR scan data for nearest wall distance across a given swept angle
        """
        min_robot_angle = self.scan_msg.min_angle
        max_robot_angle = self.scan_msg.max_angle
        increment = self.scan_msg.angle_increment

        min_dist = None
        for range,index in enumerate(self.scan_msg.ranges):
            if (min_angle < (min_robot_angle + index*increment) and max_angle > (min_robot_angle + index*increment)):
                if (not min_dist or range < min_dist):
                    min_dist = range
        return min_dist

    def drive(self, msgArg, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """   
        if not self.bumped.is_set():     
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
            self.drive(self.velocity,linear=[0,0,0],angular=[0,0,0])

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