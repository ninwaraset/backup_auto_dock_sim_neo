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
        
        self.lock_blue_pub = self.create_publisher(Float32,'/lock_blue',10)

        time_period_1 = 0.1
        self.timer_1 = self.create_timer(time_period_1,self.timer_1_callback)
        


        self.avg_vertex_theta = 0.0
        self.avg_vertex_distance = 0.0
        

        self.avg_blue_distance = 0.0
        self.avg_blue_theta = 0.0
        


        self.key_st = 0


##########################################################################################################################################################################
##########################################################################################################################################################################       
##########################################################################################################################################################################
########################################################################################################################################################################## 

    def listener_callback_1(self,msg):
        # print("msg_recived 1 : " + str(msg))
        self.avg_vertex_theta = msg.data
        
        if self.key_st == 0:
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

        
        msg_cmd_vel = Twist()
        msg_lock_blue = Float32()


        if self.key_st == 1:
            
        
            kp_1 = 0.1
            e_1 = self.avg_blue_theta
            msg_cmd_vel.angular.z = e_1*kp_1
            

            msg_lock_blue.data = 0.0

            if e_1 < 0.1 and e_1 > -0.1 :
                self.key_st = 2
                msg_cmd_vel.linear.x = 0.0
                msg_cmd_vel.angular.z = 0.0



        elif self.key_st == 2:
            
            
            kp_2_1 = 0.1
            e_2_1 = self.avg_blue_theta
            msg_cmd_vel.angular.z = e_2_1*kp_2_1
        
            kp_2_2 = 0.05
            e_2_2 = self.avg_blue_distance
            msg_cmd_vel.linear.x = e_2_2*kp_2_2
            
            msg_lock_blue.data = 0.0

            if e_2_2 < 0.1 and e_2_2 > -0.1 :
                self.key_st = 3

                msg_cmd_vel.linear.x = 0.0
                msg_cmd_vel.angular.z = 0.0


        elif self.key_st == 3:
            
        
            kp_3 = 0.1
            e_3 = self.avg_vertex_theta
            msg_cmd_vel.angular.z = e_3*kp_3

            msg_lock_blue.data = 1.0

            if e_3 < 0.1 and e_3 > -0.1 :
                self.key_st = 4
                msg_cmd_vel.linear.x = 0.0
                msg_cmd_vel.angular.z = 0.0


        
        elif self.key_st == 4:
            
            
            kp_4_1 = 0.1
            e_4_1 = self.avg_vertex_theta
            msg_cmd_vel.angular.z = e_4_1*kp_4_1
        
            kp_4_2 = 0.05
            e_4_2 = self.avg_vertex_distance -0.5
            msg_cmd_vel.linear.x = e_4_2*kp_4_2

            msg_lock_blue.data = 1.0
            
            if e_4_2 < 0.05 and e_4_2 > -0.05 :
                self.key_st = 5

                msg_cmd_vel.linear.x = 0.0
                msg_cmd_vel.angular.z = 0.0



            
        elif self.key_st == 5:

            msg_lock_blue.data = 1.0
            msg_cmd_vel.linear.x = 0.0
            msg_cmd_vel.angular.z = 0.0
            print("  GOAL !!!! ")
        
        self.cmd_publisher.publish(msg_cmd_vel)
        
        self.lock_blue_pub.publish(msg_lock_blue)
        pass






    
def main(args=None):
    
    
    rclpy.init(args=args)
    controller = MOVE()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
