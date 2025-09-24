"""
ROS2 Node to add teleop control to an autonomous robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
import termios
import tty
import sys

class SimpleTeleop(Node):
    """
    Node for passing velocity commands pased on teleop control
    to robot. Pauses function of autonomous node when actively 
    publishing to /cmd_vel.

    Attributes: 
        teleop_pub: Publisher to /use_teleop; Bool value specifies
            that NeatoFSM node should pause while teleop control
            operates
        vel_pub: Publisher to /cmd_vel topic for robot
        is_publishing (bool): internal state, specifies robot in 
            teleop mode
    """
    settings = termios.tcgetattr(sys.stdin)
    key = None

    def __init__(self):
        super().__init__('teleop_node')
        self.teleop_pub = self.create_publisher(Bool, 'use_teleop', 10)
        self.vel_pub  = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.1,self.run_loop)
        self.is_publishing = True

    def run_loop(self):
        """
        Loop to run teleop control code for the node
        Listens to keypresses in stdin and passes velocity commands
        if WASD are pressed.
        Space represents stop, and enter hands control back to 
        the autonomous node. 
        Exits on SIGINT / Ctrl + C
        """
        key = self.get_key()
        print(f"key = {key}")

        # Perform logic based on key identifier
        if key == ' ':
            msg = Bool()
            msg.data = True
            self.teleop_pub.publish(msg)
            self.is_publishing = True
            self.drive()

        if key == '\n' or key == '\r':
            print("enter")
            msg = Bool()
            msg.data = False
            self.teleop_pub.publish(msg)
            self.is_publishing = False

        # Only give velocity commands if currently set to publish
        # Otherwise, would interfere with autonomous control
        if self.is_publishing:
            if key == 'w':
                self.drive(linear=0.2)
            if key == 'a':
                self.drive(angular=0.5)
            if key == 's':
                self.drive(linear=-0.2)
            if key == 'd':
                self.drive(angular=-0.5)

        # Stop program when Ctrl C pressed
        if key == '\x03':
            raise KeyboardInterrupt
        

    def drive(self,linear=0.0,angular=0.0):
        """
        Publish velocity command to /cmd_vel
        Controls robot movement

        Args:
            linear (float): linear velocity in the x direction to
            publish to /cmd_vel, in m/s
            angular (float): angular velocity about z axis to
            publish to /cmd_vel, in rad/s
        """

        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular

        self.vel_pub.publish(twist)

    def get_key(self):
        """
        Reads a keypress from the active terminal window
        
        Returns:
            key identifier for the next key entered in stdin
        """
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main():
    rclpy.init()
    node = SimpleTeleop()
    rclpy.spin(node)
    node.drive(node.velocity,linear=0.0,angular=0.0)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
