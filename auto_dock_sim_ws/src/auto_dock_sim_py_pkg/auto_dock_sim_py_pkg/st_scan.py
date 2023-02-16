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

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import PoseWithCovarianceStamped
from matplotlib.animation import FuncAnimation
import numpy as np
from sklearn.cluster import DBSCAN
fig, ax = plt.subplots()
        
class SubscriberClass(Node):
    # print("1")

    def __init__(self):
        # print("2")
        super().__init__('state_scan')
        self.subscription_1 = self.create_subscription(LaserScan,'/scan',self.listener_callback_1,10)
   
        self.subscription_2 = self.create_subscription(String,'/nav/state',self.listener_callback_2,10)
        self.subscription_1 = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.listener_callback_3,10)


        self.cmd_publisher = self.create_publisher(String, 'topic', 10)
        self.marker_publisher = self.create_publisher(Marker, 'marker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        # self.key_1 = False
        
        self.point_from_scan = []
        self.amcl_rot = Twist()
        self.list_amcl_linear = [0.0,0.0,0.0]
        self.list_amcl_angular = [0.0,0.0,0.0]
        self.stack_x = []
        self.stack_y = []

        self.key_1 = 10
        self.key_2 = 1
        self.stack_theta = []
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
               
                # print(i)
                angle_increment = (abs(msg.angle_min - msg.angle_max))/len(msg.ranges)
                # print("angle_increment : "+str(angle_increment))
                current_angle_increment = angle_increment*(i+1)
                # print("current_angle_increment : "+str(current_angle_increment))
                current_angle = msg.angle_min + current_angle_increment
                # print("current_angle : "+str(current_angle))
                # print(math.cos(current_angle))
                point_x = float(msg.ranges[i]) * (math.cos(current_angle))
                
                
                # print((math.cos((msg.angle_min)+((msg.angle_increment)*(float(i+1))))))
                # x.append((msg.ranges)*(math.cos((msg.angle_min)+((msg.angle_increment)*(float(i+1))))))
                
                point_y = -(float(msg.ranges[i]) * (math.sin(current_angle)))
                # theta = self.list_amcl_angular[2]*math.pi
                # print("theta : "+str(theta))
                theta = -math.pi/2
                x_r = (point_x*math.cos(theta)) + (point_y*math.sin(theta))
                y_r = (point_y*math.cos(theta)) - (point_x*math.sin(theta))

                x.append(x_r)
                y.append(y_r)
                
                pass

            self.point_from_scan = [x,y]

    
            return x,y


        def lidar_DBscan(x_list,y_list,eps_value=0.04,min_samples_value=5,show_plot = 1):
            lidar_list = []
            for i in range(len(x_list)):

                lidar_list.append([x_list[i],y_list[i]])

            liadar_array = np.array(lidar_list)

            # print(type(liadar_array))
            db = DBSCAN(eps=eps_value, min_samples=min_samples_value ).fit(liadar_array)
            labels = db.labels_
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
            n_noise_ = list(labels).count(-1)

            print("Estimated number of clusters: %d" % n_clusters_)
            print("Estimated number of noise points: %d" % n_noise_)

            
            # print( "labels : "+str(db.labels_ ))
            # print(len(db.labels_))
            # print(len(x_list))
            # print(" : "+str(db.leaf_size))

            cluster_dict = {}
            # cluster_dict[-1]=[1,2]
            # cluster_dict.append("a")
            # print(cluster_dict)
            for i in range(-1,max(labels)+1):
                stack = []
                for j in range(len(labels)):
                    if labels[j] == [i]:
                        stack.append(j)
                # print(i)
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
     
        
        def check_charger(x_list,y_list, cluster_dict,threshold_base =0.50,e_base = 0.1,threshold_high = 0.15,threshold_l_r_line = 0.05):
            true_index_vertex_point_tri_of_cluster = 0
            list_dif_line = []
            dis_list = []

            # threshold_base =0.45
            # e_base = 0.07
            # threshold_high = 0.1
            # threshold_l_r_line = 0.05
            # e_high  = 0
            index_start_point_cluster = 0
            index_end_point_cluster = 0
            label_cluster_charger = 0
            index_vertex_point_tri_of_cluster = 0
            
            
            for i in range(len(cluster_dict)-1):
                
                index_start_point_cluster = cluster_dict[i][0]
                index_end_point_cluster = cluster_dict[i][-1]
                

                dif_x = (x_list[max(cluster_dict[i])]-x_list[min(cluster_dict[i])])
                pow_dif_x = dif_x**2
                dif_y = (y_list[max(cluster_dict[i])]-y_list[min(cluster_dict[i])])
                pow_dif_y = dif_y**2
                dis_xy = math.sqrt(pow_dif_x+pow_dif_y)
                print("dis_xy cluster no."+str(i)+" : "+str(dis_xy))
                
                if abs(dis_xy - threshold_base) <= e_base : 

                    dis_list.append(dis_xy)
                    center_base_x = (x_list[max(cluster_dict[i])]+x_list[min(cluster_dict[i])])/2
                    center_base_y  = (y_list[max(cluster_dict[i])]+y_list[min(cluster_dict[i])])/2
                    plt.plot(center_base_x,center_base_y,"b^")
                    
                    slope_charger = dif_y/dif_x
                    print("slope_charger : "+str(slope_charger))
                    theta_charger = math.atan(slope_charger)
                    print("theta_charger : "+str(theta_charger))


                    x_rot_z_list = []
                    y_rot_z_list = []

                    for j in range(len(cluster_dict[i])):
                        point_x = x_list[cluster_dict[i][j]]
                        point_y = y_list[cluster_dict[i][j]]
                        x_rot_z_list.append((point_x*math.cos(theta_charger)) + (point_y*math.sin(theta_charger)))
                        y_rot_z_list.append((point_y*math.cos(theta_charger)) - (point_x*math.sin(theta_charger)))

                    plt.plot(x_rot_z_list,y_rot_z_list,"cx")
                    

                    min_y_rot = y_rot_z_list[y_rot_z_list.index(min(y_rot_z_list))]
                    max_y_rot = y_rot_z_list[y_rot_z_list.index(max(y_rot_z_list))]
                    min_x_rot = x_rot_z_list[x_rot_z_list.index(min(x_rot_z_list))]

                    # print("min y : "+str(min_y_rot ))
                    # print("max y : "+str(max_y_rot))
                    plt.plot([min_x_rot,min_x_rot],[min_y_rot,max_y_rot],"m")
                    dif_y_rot = max_y_rot-min_y_rot 
                    print("dif_y_rot (high triangel measure) : "+str(dif_y_rot))
                    # if  abs(dif_y_rot - threshold_high) <= e_high :
                    if  dif_y_rot > threshold_high :

                        
                        index_vertex_point_tri_of_list = y_rot_z_list.index(max(y_rot_z_list))
                        # print("index_vertex_point_tri_of_list : "+str(index_vertex_point_tri_of_list))
                        index_vertex_point_tri_of_cluster = index_start_point_cluster+index_vertex_point_tri_of_list 
                        plt.plot(x_list[index_vertex_point_tri_of_cluster],y_list[index_vertex_point_tri_of_cluster],"wX")
                        # print("index_max_point_of_cluster : "+str(index_vertex_point_tri_of_cluster))
                        label_cluster_charger = i
                        # print("label_cluster_charger : "+str(label_cluster_charger))

                        check_line_r_dif_x = abs(x_list[index_vertex_point_tri_of_cluster]-x_list[index_end_point_cluster])
                        check_line_r_dif_y = abs(y_list[index_vertex_point_tri_of_cluster]-y_list[index_end_point_cluster])
                        check_line_r_pow_dif_x = check_line_r_dif_x**2
                        check_line_r_pow_dif_y = check_line_r_dif_y**2
                        check_line_r_dis = math.sqrt(check_line_r_pow_dif_x + check_line_r_pow_dif_y)

                        check_line_l_dif_x = abs(x_list[index_vertex_point_tri_of_cluster]-x_list[index_start_point_cluster])
                        check_line_l_dif_y = abs(y_list[index_vertex_point_tri_of_cluster]-y_list[index_start_point_cluster])
                        check_line_l_pow_dif_x = check_line_l_dif_x**2
                        check_line_l_pow_dif_y = check_line_l_dif_y**2
                        check_line_l_dis = math.sqrt(check_line_l_pow_dif_x + check_line_l_pow_dif_y)
                        # print("line l distance : "+str(check_line_l_dis))
                        # print("line r distance : "+str(check_line_r_dis))
                        line_r_l_dif = abs(check_line_l_dis-check_line_r_dis)
                        # print("difference : " + str(line_r_l_dif) )
                        plt.plot(x_list[index_end_point_cluster],y_list[index_end_point_cluster],'ks')
                        plt.plot(x_list[index_start_point_cluster],y_list[index_start_point_cluster],'ks')
                        plt.plot([x_list[index_start_point_cluster],x_list[index_vertex_point_tri_of_cluster]],[y_list[index_start_point_cluster],y_list[index_vertex_point_tri_of_cluster]],'m')
                        plt.plot([x_list[index_end_point_cluster],x_list[index_vertex_point_tri_of_cluster]],[y_list[index_end_point_cluster],y_list[index_vertex_point_tri_of_cluster]],'m')
                        
                        if abs(line_r_l_dif) <= threshold_l_r_line :
                            list_dif_line.append(line_r_l_dif)
                            true_index_vertex_point_tri_of_cluster = index_vertex_point_tri_of_cluster



                # plt.plot(cluster_dict[i][index_vertex_point_tri_of_list])
                line_dis_x = [x_list[min(cluster_dict[i])], x_list[max(cluster_dict[i])]]
                line_dis_y = [y_list[min(cluster_dict[i])], y_list[max(cluster_dict[i])]]
                # print(x_list[min(cluster_dict[i])])
                # print(x_list[0])
                # print(line_dis_x)
                # print(line_dis_y)
                plt.plot(line_dis_x,line_dis_y,"y")
                
            # true_index_vertex_point_tri_of_cluster0
            # print("dis_list"+str(dis_list))
            return true_index_vertex_point_tri_of_cluster,label_cluster_charger,list_dif_line


        def cal_theta_distance(x_list,y_list,idx_vtx_tri,origin_x=0,origin_y=0):
            origin_dif_x = abs(origin_x - x_list[idx_vtx_tri])
            origin_dif_y = abs(origin_y - y_list[idx_vtx_tri])
            origin_pow_dif_x = origin_dif_x**2
            origin_pow_dif_y = origin_dif_y**2
            dis_origin = math.sqrt(origin_pow_dif_x + origin_pow_dif_y)
            slope_origin = origin_dif_y/origin_dif_x 
            theta_origin = math.atan(slope_origin)
            return dis_origin,theta_origin
        

        def split_r_l_charger(x_list,y_list,cluster_dict,label_charger,idx_vtx_tri):
            plt.plot(x_list[idx_vtx_tri],y_list[idx_vtx_tri],"r^")
            fig = plt.gcf()
            ax = fig.gca()
            circle1 = plt.Circle((x_list[idx_vtx_tri],y_list[idx_vtx_tri]), 0.2, color='m', fill=False)
            # circle3 = plt.Circle((x_list[idx_vtx_tri],y_list[idx_vtx_tri]), 0.35, color='g', fill=False)
            
            # ax = plt.gca()
            # ax.cla() # clear things for fresh plot
            ax.add_patch(circle1)
            # ax.add_patch(circle3)
            
            list_idx_charger = cluster_dict[label_charger]
            itls_idx_list = []
            for i in range(len(list_idx_charger)):
                dif_x = x_list[idx_vtx_tri]-x_list[list_idx_charger[i]]
                dif_y = y_list[idx_vtx_tri]-y_list[list_idx_charger[i]]
                pow_dif_x = dif_x**2
                pow_dif_y = dif_y**2
                dis_xy = math.sqrt(pow_dif_x+pow_dif_y)
                if dis_xy <= 0.2 :
                    itls_idx_list.append(list_idx_charger[i])

            # print( itls_idx_list)
            r_idx_list = []
            l_idx_list = []
            point_r_x = []
            point_r_y = []
            point_l_x = []
            point_l_y = []

            for i in range(len(itls_idx_list)):
                if itls_idx_list[i] >= idx_vtx_tri:
                    r_idx_list.append( itls_idx_list[i])
                    point_r_x.append(x_list[itls_idx_list[i]])
                    point_r_y.append(y_list[itls_idx_list[i]])
                if itls_idx_list[i] <= idx_vtx_tri:
                    l_idx_list.append( itls_idx_list[i])
                    point_l_x.append(x_list[itls_idx_list[i]])
                    point_l_y.append(y_list[itls_idx_list[i]])
                    
            # print("R : "+str(r_idx_list))
            # print("L : "+str(l_idx_list))
            # print(r_idx_list[-1])

            ################## will change linear reg for real line *fix
            line_r_x = [x_list[r_idx_list[0]],x_list[r_idx_list[-1]]]
            line_r_y = [y_list[r_idx_list[0]],y_list[r_idx_list[-1]]]
            # line_r_y = [y_list[r_idx_list[0]],y_list[idx_vtx_tri]]

            line_l_x = [x_list[l_idx_list[0]],x_list[l_idx_list[-1]]]
            line_l_y = [y_list[l_idx_list[0]],y_list[l_idx_list[-1]]]
            # plt.xlim([-1.5, 1.5])
            # plt.ylim([-0.5,1.5])
            # plt.show()
            # plt.cla()
            # plt.plot(point_r_x,point_r_y,"g^")
            # plt.plot(point_l_x,point_l_y,"y*")
            plt.plot(line_r_x,line_r_y,'r')
            plt.plot(line_l_x,line_l_y,'b')
            
            line_base_clean_x = [x_list[l_idx_list[0]],x_list[r_idx_list[-1]]]
            line_base_clean_y = [y_list[l_idx_list[0]],y_list[r_idx_list[-1]]]
            plt.plot(line_base_clean_x,line_base_clean_y,'g')
            center_base_clean_x = sum( line_base_clean_x)/2
            center_base_clean_y = sum( line_base_clean_y)/2
            plt.plot(center_base_clean_x,center_base_clean_y,'k*')
            
            plt.plot([x_list[idx_vtx_tri],center_base_clean_x],[y_list[idx_vtx_tri],center_base_clean_y],'g')
            slope_vg = (y_list[idx_vtx_tri]-center_base_clean_y)/(x_list[idx_vtx_tri]-center_base_clean_x)
            theta_vg = math.atan(slope_vg)
            print("--> slope_vg : "+str(slope_vg))
            print("--> theta_vg : "+str(theta_vg))

            origin_x = 0
            origin_y = 0

            vertex_x = x_list[idx_vtx_tri]
            vertex_y = y_list[idx_vtx_tri]

            tran_x = vertex_x
            tran_y = vertex_y

            line_1_x = [origin_x,tran_x]
            line_1_y = [origin_y,tran_y]


            # theta_rot = -math.pi*2/3
            # dist_rot = 1
            if theta_vg >= 0 :
                theta_rot = theta_vg - math.pi
            else :
                theta_rot =  theta_vg
            dist_rot = 0.35
            circle2 = plt.Circle((x_list[idx_vtx_tri],y_list[idx_vtx_tri]), dist_rot,ls = "--", color='g', fill=False)
            ax.add_patch(circle2)

            blue_x = dist_rot*math.cos(theta_rot) + tran_x
            blue_y = dist_rot*math.sin(theta_rot) + tran_y
            print("--> blue_point : "+str([blue_x,blue_y]))

            line_2_x = [tran_x,blue_x]
            line_2_y = [tran_y,blue_y]

            # plt.plot(origin_x,origin_y,marker="o",color="g")
            # plt.plot(tran_x,tran_y,marker="^",color="r")
            # plt.plot(line_1_x,line_1_y,ls = "-", color='y')

            plt.plot(blue_x,blue_y,marker="*",color="b")
            plt.plot(line_2_x,line_2_y,ls = "-", color='m')
            # plt.show()
            return r_idx_list , l_idx_list
        
        def traj_point(x_list,y_list,idx_vtx_tri,dis_origin,theta_origin):
            traj_base_x = dis_origin * math.cos(theta_origin)
            traj_base_y = 0
            traj_high_x = traj_base_x + 0
            traj_high_y = traj_base_y + (dis_origin * math.sin(theta_origin))
            
            # plt.plot([0,traj_base_x],[0,traj_base_y],'k')
            # plt.plot([traj_base_x,traj_high_x],[traj_base_y,traj_high_y],'k')
            
            
            
            
            
##########################################################################################################################################################################
##########################################################################################################################################################################       
##########################################################################################################################################################################
##########################################################################################################################################################################  
        def call_scan():
            x_list,y_list = polar_to_xy()
            # x_list = x_list[89:631]
            # y_list = y_list[89:631]

            cluster_dict = lidar_DBscan(x_list,y_list,eps_value=0.04,min_samples_value=5)
            # print(cluster_dict)
            idx_vtx_tri,label_charger,list_dif_line = check_charger(x_list,y_list, cluster_dict)
            plt.plot(0,0,"r^")
            plt.plot([0,x_list[idx_vtx_tri]],[0,y_list[idx_vtx_tri]],"b")
            print(list_dif_line)
            
            plt.plot(x_list[idx_vtx_tri],y_list[idx_vtx_tri],"r^")
            r,t = cal_theta_distance(x_list,y_list,idx_vtx_tri)
            print(t)
            t = t-(math.pi)/2
            print(t)

            print("xy : ",[x_list[idx_vtx_tri],y_list[idx_vtx_tri]])

            print("distance ,theta(degree) : ",[r,((t*180)/math.pi)])
            print("distance ,theta(radian) : ",[r,t])
            
            x_cal =r*math.cos(t) 
            y_cal = r*math.sin(t)
            
            goal_x = x_list[idx_vtx_tri]
            goal_y = y_list[idx_vtx_tri]
            # print("xy cal : ",[x_cal,y_cal])
            # plt.plot(x_cal,y_cal,"r*")
            dis_origin,theta_origin = cal_theta_distance(x_list,y_list,idx_vtx_tri)
            split_r_l_charger(x_list,y_list,cluster_dict,label_charger,idx_vtx_tri)
            traj_point(x_list,y_list,idx_vtx_tri,dis_origin,theta_origin)
            if list_dif_line == [] :
                print("ERROR NOT FONUD -----------------------------------------------------------------------------------")
            else:
                print("FOUNDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                self.stack_theta.append(t*180/math.pi)
                self.key_1 -= 1
                # print(self.key_1)
            plt.show()

        # call_scan
        # plt.plot(x[index_vertex_point_tri_of_cluster],y[index_vertex_point_tri_of_cluster],"ro")
        
        # # pass
        if self.key_1 > 0:
            print("scan") 
            call_scan()
            
        elif self.key_1 == 0:
            if self.key_2 > 0:
                print(self.stack_theta)
                print((sum(self.stack_theta)/10))
                
                self.key_2 = 0

    def listener_callback_2(self, msg):
        # print(msg.data)
        if self.key_1 == False:
            print(msg.data)
        if msg.data == 'success':
            self.key_1 = True
        pass


    def listener_callback_3(self, msg):
        pass
        # self.list_amcl_linear[0] = msg.pose.pose.position.x
        # self.list_amcl_linear[1] = msg.pose.pose.position.y
        # self.list_amcl_linear[2] = msg.pose.pose.position.z
        # self.list_amcl_angular[0] = msg.pose.pose.orientation.x
        # self.list_amcl_angular[1] = msg.pose.pose.orientation.y
        # self.list_amcl_angular[2] = msg.pose.pose.orientation.z

        # print("pose current : "+str(self.list_amcl_linear)+str(self.list_amcl_angular))



        
    
    def timer_callback(self):

        pass
        


def main(args=None):
    # print("3")
    rclpy.init(args=args)

    subscriber = SubscriberClass()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    # print("4")
    
    main()