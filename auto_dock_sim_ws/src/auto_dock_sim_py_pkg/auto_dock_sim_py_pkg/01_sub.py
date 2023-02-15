#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SubscriberClass(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            # '/amcl_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        dcm = 4
        print("tran x : " +str(round(msg.pose.pose.position.x,dcm)))
        print("tran y : " +str(round(msg.pose.pose.position.y,dcm)))
        print("rot z : " +str(round((msg.pose.pose.orientation.z)*180,dcm)))
        print("++++++++++++++++++++++++++++++++++++++++++++++++")




def main(args=None):
    rclpy.init(args=args)

    subscriber = SubscriberClass()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

