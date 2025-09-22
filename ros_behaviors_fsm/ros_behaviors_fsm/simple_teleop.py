"""

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
from pynput.keyboard import Key, Listener, KeyCode

import termios
import tty
import sys

class SimpleTeleop(Node):

    settings = termios.tcgetattr(sys.stdin)
    key = None

    def __init__(self):
        super().__init__('teleop_node')
        self.teleop_pub = self.create_publisher(Bool, 'use_teleop', 10)
        self.vel_pub  = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.1,self.run_loop)
        self.is_publishing = True

    def run_loop(self):
        key = self.get_key()
        print(f"key = {key}")
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
        if self.is_publishing:
            if key == 'w':
                self.drive(linear=0.2)
            if key == 'a':
                self.drive(angular=0.5)
            if key == 's':
                self.drive(linear=-0.2)
            if key == 'd':
                self.drive(angular=-0.5)
        if key == '\x03':
            raise KeyboardInterrupt
        

    def drive(self,linear=0.0,angular=0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular

        self.vel_pub.publish(twist)

    def get_key(self):
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

    
