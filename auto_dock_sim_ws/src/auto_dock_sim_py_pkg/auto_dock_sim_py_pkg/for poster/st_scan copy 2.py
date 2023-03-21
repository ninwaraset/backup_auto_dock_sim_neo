#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
# import xlrd
# from xlrd import open_workbook
# import xlwt
from tempfile import TemporaryFile
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon

from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import PoseWithCovarianceStamped
from matplotlib.animation import FuncAnimation
import numpy as np
from sklearn.cluster import DBSCAN
fig, ax = plt.subplots()
        
class SCAN(Node):
    # print("1")

    def __init__(self):
        # print("2")
        super().__init__('state_scan')
        self.subscription_1 = self.create_subscription(LaserScan,'/scan',self.listener_callback_1,10)


        
        self.vertex_distance_publisher = self.create_publisher(Float32,'/vertex_distance',10)
        self.vertex_theta_publisher = self.create_publisher(Float32,'/vertex_theta',10)
        
        self.blue_distance_publisher = self.create_publisher(Float32,'/blue_distance',10)
        self.blue_theta_publisher = self.create_publisher(Float32,'/blue_theta',10)

        self.main_pub = self.create_publisher(Float32,'/main',10)

        time_period_pub = 0.1
        self.timer = self.create_timer(time_period_pub,self.timer_callback)


    
        # self.subscription_2 = self.create_subscription(String,'/nav/state',self.listener_callback_2,10)
        # self.subscription_3 = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.listener_callback_3,10)


        # self.cmd_publisher = self.create_publisher(String, 'topic', 10)
        # self.marker_publisher = self.create_publisher(Marker, 'marker', 10)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        # self.key_1 = False
        
        # self.point_from_scan = []
        # self.amcl_rot = Twist()
        # self.list_amcl_linear = [0.0,0.0,0.0]
        # self.list_amcl_angular = [0.0,0.0,0.0]
        # self.stack_x = []
        # self.stack_y = []
        self.round_scan_init = 1
        self.round_scan = self.round_scan_init
        self.key_avg_scan = 1
        self.stack_theta_vertex = []
        self.stack_distance_vertex = []
        self.stack_theta_blue = []
        self.stack_distance_blue = []
        

        self.avg_vertex_distance = 0.0
        self.avg_vertex_theta = 0.0

        self.avg_blue_distance = 0.0
        self.avg_blue_theta = 0.0
        


    def listener_callback_1(self, msg):
        
        
##########################################################################################################################################################################
##########################################################################################################################################################################       
##########################################################################################################################################################################
##########################################################################################################################################################################      
        def polar_to_xy():
            x = []
            y = []
            i = 0
            for i in range(len(msg.ranges)):
                angle_increment = (abs(msg.angle_min - msg.angle_max))/len(msg.ranges)

                current_angle_increment = angle_increment*(i+1)

                current_angle = msg.angle_min + current_angle_increment

                point_x = float(msg.ranges[i]) * (math.cos(current_angle))
                point_y = -(float(msg.ranges[i]) * (math.sin(current_angle)))

                theta = -math.pi/2
                x_r = (point_x*math.cos(theta)) + (point_y*math.sin(theta))
                y_r = (point_y*math.cos(theta)) - (point_x*math.sin(theta))

                x.append(x_r)
                y.append(y_r)

            return x,y


        def lidar_DBscan(x_list,y_list,eps_value=0.04,min_samples_value=5,show_plot = 1):
            print("\n++ DB SCAN ++")
            lidar_list = []
            for i in range(len(x_list)):

                lidar_list.append([x_list[i],y_list[i]])

            liadar_array = np.array(lidar_list)

            db = DBSCAN(eps=eps_value, min_samples=min_samples_value ).fit(liadar_array)
            labels = db.labels_
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
            n_noise_ = list(labels).count(-1)

            print(" --> Estimated number of clusters: %d" % n_clusters_)
            print(" --> Estimated number of noise points: %d" % n_noise_)

            cluster_dict = {}

            for i in range(-1,max(labels)+1):
                stack = []
                for j in range(len(labels)):
                    if labels[j] == [i]:
                        stack.append(j)

                cluster_dict[i]=stack
            
            if show_plot == 1:
                unique_labels = set(labels)
                core_samples_mask = np.zeros_like(labels, dtype=bool)
                core_samples_mask[db.core_sample_indices_] = True

                colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
                for k, col in zip(unique_labels, colors):
                    if k == -1:
                        # Black used for noise.
                        col = [0, 0, 0, 1]

                    class_member_mask = labels == k

                    xy = liadar_array[class_member_mask & core_samples_mask]
                    plt.plot(
                        xy[:, 0],
                        xy[:, 1],
                        "o",
                        markerfacecolor=tuple(col),
                        markeredgecolor="k",
                        markersize=14,
                    )

                    xy = liadar_array[class_member_mask & ~core_samples_mask]
                    plt.plot(
                        xy[:, 0],
                        xy[:, 1],
                        "o",
                        markerfacecolor=tuple(col),
                        markeredgecolor="k",
                        markersize=6,
                    )
                # plt.title(f"Estimated number of clusters: {n_clusters_}")
                ### plt.show()
            return cluster_dict 
     
        

##########################################################################################################################################################################
##########################################################################################################################################################################       
##########################################################################################################################################################################
##########################################################################################################################################################################  
        def call_scan():
            print("\n!! CALL SCAN FUNCTION !!")
            x_list,y_list = polar_to_xy()

            cluster_dict = lidar_DBscan(x_list,y_list,eps_value=0.04,min_samples_value=5)

            print(cluster_dict)
            plt.show()

        # call_scan
        # plt.plot(x[index_vertex_point_tri_of_cluster],y[index_vertex_point_tri_of_cluster],"ro")
        
        # # pass
        if self.round_scan > 0:
            # print("SCAN") 
            print("---------------------------------------------------------------------------------------------------------------------------------")
            print("\n>o| scan round : "+str((self.round_scan_init+1)-self.round_scan))
            call_scan()
            
        elif self.round_scan == 0:
            if self.key_avg_scan > 0:
                print("\n ----- avg ----- \n")
                print("stack_theta_vertex : \n"+str(self.stack_theta_vertex))
                self.avg_vertex_distance = (sum(self.stack_theta_vertex)/self.round_scan_init)
                print("avg stack_theta_vertex: "+str(self.avg_vertex_distance))
                
                print("")
                print("stack_distance_vertex : \n"+str(self.stack_distance_vertex))
                self.avg_vertex_theta =(sum(self.stack_distance_vertex)/self.round_scan_init)
                print("avg stack_distance_vertex: "+str(self.avg_vertex_theta))

                print("")
                print("stack_theta_blue : \n"+str(self.stack_theta_blue))
                self.avg_blue_distance = (sum(self.stack_theta_blue)/self.round_scan_init)
                print("avg stack_theta_blue : "+str(self.avg_blue_distance))

                print("")
                print("self.stack_distance_blue : \n"+str(self.stack_distance_blue))
                self.avg_blue_theta = (sum(self.stack_distance_blue)/self.round_scan_init)
                print("avg self.stack_distance_blue : "+str(self.avg_blue_theta))

                plt.show()
                
                self.key_avg_scan = 0


    def listener_callback_2(self, msg):
        # print(msg.data)
        if self.key_1 == False:
            print(msg.data)
        if msg.data == 'success':
            self.key_1 = True
        pass






        
    
    def timer_callback(self):
        msg_vertex_distance= Float32()
        msg_vertex_theta= Float32()
        msg_blue_distance = Float32()
        msg_blue_theta = Float32()
        

        msg_vertex_distance.data = self.avg_vertex_distance
        msg_vertex_theta.data = self.avg_vertex_theta
        
        msg_blue_distance.data = self.avg_blue_distance
        msg_blue_theta.data = self.avg_blue_theta

        self.vertex_distance_publisher.publish(msg_vertex_distance)
        self.vertex_theta_publisher.publish(msg_vertex_theta)
        self.blue_distance_publisher.publish(msg_blue_distance)
        self.blue_theta_publisher.publish(msg_blue_theta)

        msg_main = Float32()
        msg_main.data = 0.0
        self.main_pub.publish(msg_main)
        pass
        


def main(args=None):
    # print("3")
    rclpy.init(args=args)

    subscriber = SCAN()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    # print("4")
    
    main()