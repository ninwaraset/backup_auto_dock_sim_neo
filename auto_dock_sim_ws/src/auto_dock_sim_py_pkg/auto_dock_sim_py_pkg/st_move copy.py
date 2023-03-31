#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class Dummy(Node):
    def __init__(self):
        super().__init__('node_name')
        self.cmd_publisher = self.create_publisher(Twist,'/cmd_vel',10)

        self.subscription_2 = self.create_subscription(Float32,'/vertex_theta',self.listener_callback_1,10)
        self.subscription_1 = self.create_subscription(Float32,'/vertex_distance',self.listener_callback_2,10)
        self.subscription_4 = self.create_subscription(Float32,'/blue_theta',self.listener_callback_3,10)
        self.subscription_3 = self.create_subscription(Float32,'/blue_distance',self.listener_callback_4,10)
        

        self.subscription_main = self.create_subscription(Float32,'/main',self.listener_callback_main,10)


        time_period_1 = 0.1
        self.timer_1 = self.create_timer(time_period_1,self.timer_1_callback)
        
        self.repeat_pub = self.create_publisher(Float32,'/repeat',10)
        # self.timer = self.create_timer(0.1,self.timer_calback)

        # self.repeat_recive_sub = self.create_subscription(Float32,'/repeat_c2',self.listener_callback_repeat,10)
        self.key_repeat = False

        # print("st_move")

        # distance_angular_move1 =  -0.47099855243640937

        # distance_linear_move1 = 1.1163295358780199

        # speed_move1 = 0.05
        # self.move(type_move="angular",distance=distance_angular_move1,speed=speed_move1)
        # self.move(type_move="linear",distance=distance_linear_move1,speed=speed_move1)
        # self.move(type_move="angular",distance=-distance_angular_move1,speed=speed_move1)
        self.avg_vertex_distance = 0.0
        
        self.avg_vertex_theta = 0.0

        self.avg_blue_distance = 0.0
        self.avg_blue_distance_old = 0.0

        self.avg_blue_theta = 0.0
        self.avg_blue_theta_old = 0.0


        # Dummy().destroy_node()
        # rclpy.shutdown()
        # self.timer_period = 0.5
        # self.timer = self.create_timer(self.timer_period,self.timer_calback)
        # self.num = self.timer_period
        # # clock = Clock()
        # now = Clock().now()
        # self.time1 = now.nanoseconds
        

        self.key_1 = 1
        self.key_st = 1

##########################################################################################################################################################################
##########################################################################################################################################################################       
##########################################################################################################################################################################
########################################################################################################################################################################## 
    def move(self,type_move="linear",distance = 0,speed=0):
            print("type_move : "+str(type_move))
            print("fn move")
            distance = float(distance)
            speed = float(speed)
            if distance < 0 :
                speed = - speed
            print("distance : "+str(distance))
            print("speed : "+str(speed))
            # print(""+str())

            current_distance = 0
            # print("current_distance : "+str(current_distance))

            msg = Twist()
            # 
            
            if type_move == "linear":
                msg.linear.x = speed
                msg.angular.z = 0.0
                pass
            elif type_move == "angular":
                msg.linear.x = 0.0
                msg.angular.z = speed
                pass
            # msg.linear.x = 1.0

            t_i = Clock().now().nanoseconds/((10**9))
            print("t_i : "+str(t_i))
            time_d = 0
            while(abs(current_distance) <abs(distance)):
                # print("moving")
                # print("current_distance"+str(current_distance))
                #Publish the velocity
                
                self.cmd_publisher.publish(msg)
                #Takes actual time to velocity calculus
                t_c=  Clock().now().nanoseconds/((10**9))
                time_d = t_c-t_i
                #Calculates distancePoseStamped
                current_distance= speed*(time_d)
                
                #After the loop, stops the robot
            # msg.linear.x = 0.0

            msg.angular.z = 0.0
            msg.linear.x = 0.0
            #Force the robot to stop
            # self.cmd_publisher.publish(msg)
            # print("time_d : "+str(time_d))
            # print("current_distance : "+str(current_distance))
            
            
##########################################################################################################################################################################
##########################################################################################################################################################################       
##########################################################################################################################################################################
########################################################################################################################################################################## 
        # move(type_move="linear",distance=1,speed=0.1)
        
    # def timer_calback(self):

    #     msg = Twist()
    #     msg.linear.x = 1.0
    #     msg.angular.z = 1.0
    #     # self.cmd_publisher.publish(msg)
    #     print("hello mother fucker"+str(self.num))
    #     self.num += self.timer_period
    #     now = Clock().now()
    #     seconds = now.nanoseconds/(10**(-9))
    #     nanoseconds = (now.nanoseconds-self.time1 )/(10**(9))
    #     print(f"Current ROS 2 system time: "+str(nanoseconds)+" seconds")
    def listener_callback_1(self,msg):
        # print("msg_recived 2 : " + str(msg))
        self.avg_vertex_theta = msg.data

        pass

    def listener_callback_2(self,msg):
        # print("msg_recived 1 : " + str(msg))
        self.avg_vertex_distance = msg.data
        
        pass
    

    def listener_callback_3(self,msg):
        # print("msg_recived 4 : " + str(msg))
        self.avg_blue_theta = msg.data
        pass

    def listener_callback_4(self,msg):
        # print("msg_recived 3 : " + str(msg))
        self.avg_blue_distance = msg.data
        pass

    

    def listener_callback_main(self,msg):
        print("st_move")

    def timer_1_callback(self):


        print("avg stack_theta_vertex    :  "+str(self.avg_vertex_theta))
        print("avg stack_distance_vertex :  "+str(self.avg_vertex_distance))
        print("avg stack_theta_blue      :  "+str(self.avg_blue_theta))
        print("avg stack_distance_blue   :  "+str(self.avg_blue_distance))
        print("")
        kp_1 = 0.1
        e_1 = self.avg_blue_theta
        msg = Twist()
        msg.angular.z = e_1*kp_1
        self.cmd_publisher.publish(msg)

        
        pass



    
def main(args=None):
    
    
    rclpy.init(args=args)
    controller = Dummy()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
