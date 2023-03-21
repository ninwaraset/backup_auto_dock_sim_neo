#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math 
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
        self.i = 0
        self.list_x = []
        self.list_y = []



    def listener_callback(self, msg):
        dcm = 4
        t_x = round(msg.pose.pose.position.x,dcm)
        t_y = round(msg.pose.pose.position.y,dcm)
        r_z = round((msg.pose.pose.orientation.z),dcm)
        self.list_x.append(t_x)
        self.list_y.append(t_y)
        if t_x == 0:
            t_x = 0.00000000000000000000000000000000000000000000000000000000000000000000000000001
        
        r_z_cal = (math.atan( t_y / t_x ))/(math.pi/2)
        print(r_z_cal)
        print("tran x : " +str(t_x))
        print("tran y : " +str(t_y))
        print("rot z : " +str(r_z))
        print("rot z cal : " +str(r_z_cal))
        print("++++++++++++++++++++++++++++++++++++++++++++++++")
        # plt.plot(t_x,t_y,"rd")
        plt.plot(self.i,r_z,"gd")
        plt.plot(self.i,r_z_cal,"bd")

        
        self.i += 1
        if self.i > 1000 :
            # print("max x")
            # print(max(self.list_x))
            # print("min x")
            # print(min(self.list_x))
            # print("max y")
            # print(max(self.list_y))
            # print("min y")
            # print(min(self.list_y))

            # plt.plot(max(self.list_x)-((max(self.list_x)-min(self.list_x))/2),max(self.list_y)-((max(self.list_y)-min(self.list_y))/2),"bX")
            # print(max(self.list_x)-((max(self.list_x)-min(self.list_x))/2),max(self.list_y)-((max(self.list_y)-min(self.list_y))/2))
            plt.show()

            dasfsd



def main(args=None):
    rclpy.init(args=args)

    subscriber = SubscriberClass()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

