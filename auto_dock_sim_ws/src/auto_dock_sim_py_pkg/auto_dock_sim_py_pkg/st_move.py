#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class MOVE(Node):
    def __init__(self):
        super().__init__('node_name')
        
        self.cmd_publisher = self.create_publisher(Twist,'/cmd_vel',10)

        self.subscription_2 = self.create_subscription(Float32,'/vertex_theta',self.listener_callback_1,10)
        self.subscription_1 = self.create_subscription(Float32,'/vertex_distance',self.listener_callback_2,10)
        self.subscription_4 = self.create_subscription(Float32,'/blue_theta',self.listener_callback_3,10)
        self.subscription_3 = self.create_subscription(Float32,'/blue_distance',self.listener_callback_4,10)
        


        time_period_1 = 0.1
        self.timer_1 = self.create_timer(time_period_1,self.timer_1_callback)
        


        self.avg_vertex_theta = 99
        self.avg_vertex_distance = 99
        

        self.avg_blue_distance = 99
        self.avg_blue_theta = 99
        


        self.key_st = 0


##########################################################################################################################################################################
##########################################################################################################################################################################       
##########################################################################################################################################################################
########################################################################################################################################################################## 

    def listener_callback_1(self,msg):
        # print("msg_recived 1 : " + str(msg))
        self.avg_vertex_theta = msg.data
        self.key_st = 1
        pass

    def listener_callback_2(self,msg):
        # print("msg_recived 2 : " + str(msg))
        self.avg_vertex_distance = msg.data
        pass
    

    def listener_callback_3(self,msg):
        # print("msg_recived 2 : " + str(msg))
        self.avg_blue_theta = msg.data
        pass

    def listener_callback_4(self,msg):
        # print("msg_recived 4 : " + str(msg))
        self.avg_blue_distance = msg.data
        pass

    
##########################################################################################################################################################################
##########################################################################################################################################################################       
##########################################################################################################################################################################
########################################################################################################################################################################## 


    def timer_1_callback(self):

        
        print("avg stack_theta_vertex    :  "+str(self.avg_vertex_theta))
        print("avg stack_distance_vertex :  "+str(self.avg_vertex_distance))
        print("avg stack_theta_blue      :  "+str(self.avg_blue_theta))
        print("avg stack_distance_blue   :  "+str(self.avg_blue_distance))
        print("")
        print("st : " + str(self.key_st))

        if self.key_st == 1:

            kp_1 = 0.1
            e_1 = self.avg_blue_theta
            msg = Twist()
            msg.angular.z = e_1*kp_1
            self.cmd_publisher.publish(msg)
            if e_1 < 0.05 and e_1 > -0.05 :
                self.key_st = 2


        elif self.key_st == 2:
            pass
            kp_2_1 = 0.1
            e_2_1 = self.avg_blue_theta
            msg = Twist()
            msg.angular.z = e_2_1*kp_2_1
            self.cmd_publisher.publish(msg)

            kp_2_2 = 0.05
            e_2_2 = self.avg_blue_distance
            msg = Twist()
            msg.linear.x = e_2_2*kp_2_2
            self.cmd_publisher.publish(msg)
            if e_2_2 < 0.05 and e_2_2 > -0.05 :
                self.key_st = 3
        pass






    
def main(args=None):
    
    
    rclpy.init(args=args)
    controller = MOVE()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
