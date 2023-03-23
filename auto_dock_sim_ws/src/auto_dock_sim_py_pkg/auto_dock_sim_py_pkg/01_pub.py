#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Dummy(Node):
    def __init__(self):
        super().__init__('node_name')
        self.repeat_pub = self.create_publisher(Float32,'/repeat',10)
        self.timer = self.create_timer(0.1,self.timer_calback)

        self.repeat_recive_sub = self.create_subscription(Float32,'/repeat_c2',self.listener_callback,10)
        self.key_repeat = False

    def timer_calback(self):
        msg = Float32()
        
        if self.key_repeat :
            msg.data = 0.0
        else :
            msg.data = 1.0
        # msg = Twist()
        # msg.linear.x = 1.0
        # msg.angular.z = 1.0
        self.repeat_pub.publish(msg)
        print("call "+ str(msg.data))

    def listener_callback(self,msg):
        print("listen")
        if msg.data == 1.0:
            self.key_repeat = True
        



        

def main(args=None):
    
    rclpy.init(args=args)
    controller = Dummy()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()



if __name__=='__main__':
    main()
