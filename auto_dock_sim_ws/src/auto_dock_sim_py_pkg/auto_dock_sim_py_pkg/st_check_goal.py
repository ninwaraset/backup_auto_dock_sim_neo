#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import numpy
class SubscriberClass(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_1 = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.listener_callback_1,10)
   
        self.subscription_2 = self.create_subscription(Twist,'/nav/goal_pose',self.listener_callback_2,10)
        # self.cmd_publisher = self.create_publisher(String,'/nav/state',10)
        self.cmd_publisher = self.create_publisher(String, '/nav/state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.amcl = Twist()
        self.goal = Twist()
        self.key_1 = False
        self.key_2 = False
        self.list_amcl_linear = [0.0,0.0,0.0]
        self.list_amcl_angular = [0.0,0.0,0.0]
        self.list_goal_linear = [0.0,0.0,0.0]
        self.list_goal_angular = [0.0,0.0,0.0]
        self.list_result_linear = [0.0,0.0,0.0]
        self.list_result_angular = [0.0,0.0,0.0]

    def listener_callback_1(self, msg):
        # msg = msg.pose.pose
        # self.amcl.linear.x = msg.pose.pose.position.x
        # self.amcl.linear.y = msg.pose.pose.position.y
        # self.amcl.linear.z = msg.pose.pose.position.z
        self.list_amcl_linear[0] = msg.pose.pose.position.x
        self.list_amcl_linear[1] = msg.pose.pose.position.y
        self.list_amcl_linear[2] = msg.pose.pose.position.z
        self.list_amcl_angular[0] = msg.pose.pose.orientation.x
        self.list_amcl_angular[1] = msg.pose.pose.orientation.y
        self.list_amcl_angular[2] = msg.pose.pose.orientation.z
        # print(self.odom)
        self.key_1 = True
        # print(self.list_amcl)
        # print('amcl : '+str(self.amcl.linear.x))
        print("pose current : "+str(self.list_amcl_linear)+str(self.list_amcl_angular))
        
        

    def listener_callback_2(self, msg):
        # self.goal.linear.x = msg.linear.x
        # self.goal.linear.y = msg.linear.y
        # self.goal.linear.z = msg.linear.z
        self.list_goal_linear[0] = msg.linear.x
        self.list_goal_linear[1] = msg.linear.y
        self.list_goal_linear[2] = msg.linear.z
        self.list_goal_angular[0] = msg.angular.x
        self.list_goal_angular[1] = msg.angular.y
        self.list_goal_angular[2] = msg.angular.z
        # print(1)
        self.key_2 = True
        # print('goal : '+str(self.goal.linear.x))
        # print("send goal : "+str(self.list_goal_linear)+str(self.list_goal_angular))
        
        
        # print(self.goal)
    def timer_callback(self):
        self.list_result_linear[0] = self.list_goal_linear[0] - self.list_amcl_linear[0]
        self.list_result_linear[1] = self.list_goal_linear[1] - self.list_amcl_linear[1]
        self.list_result_linear[2] = self.list_goal_linear[2] - self.list_amcl_linear[2]
        self.list_result_angular[0] = self.list_goal_angular[0] - self.list_amcl_angular[0] 
        self.list_result_angular[1] = self.list_goal_angular[1] - self.list_amcl_angular[1]
        self.list_result_angular[2] = self.list_goal_angular[2] - self.list_amcl_angular[2] 
        # print(self.list_goal_angular)
        # print(self.list_amcl_angular)
        # print(self.list_amcl_linear)
        # print(self.list_goal_linear)
        msg = String()
        msg.data = 'waiting move to goal'
        
        if self.key_1 :
            if self.key_2:
                
                print("result linear :" + str(self.list_result_linear))
              
                pass
                
                if abs(self.list_result_linear[0]) < 0.5:
                    msg.data = 'success'
                #     print('success')
                    # self.key_1 = False
                    #  self.key_2 = False
        self.cmd_publisher.publish(msg)
# def goal():
#     SubscriberClass.check_goal
    

def main(args=None):
    rclpy.init(args=args)

    subscriber = SubscriberClass()
    rclpy.spin(subscriber)
    
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

