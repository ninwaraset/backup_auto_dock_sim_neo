#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.clock import Clock

class Dummy(Node):
    def __init__(self):
        super().__init__('node_name')
        self.cmd_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        # self.cmd_publisher = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        # self.subscription_1 = self.create_subscription(LaserScan,'/scan',self.listener_callback_1,10)

        print("st_move")
        self.move(type_move="linear",distance=1,speed=0.5)
        # self.timer_period = 0.5
        # self.timer = self.create_timer(self.timer_period,self.timer_calback)
        # self.num = self.timer_period
        # # clock = Clock()
        # now = Clock().now()
        # self.time1 = now.nanoseconds

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
            print("current_distance : "+str(current_distance))

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
            self.cmd_publisher.publish(msg)
            print("time_d : "+str(time_d))
            print("current_distance : "+str(current_distance))
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
        print("msg_recived : " + str(msg))
        pass
        



def main(args=None):
    
    
    rclpy.init(args=args)
    controller = Dummy()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
