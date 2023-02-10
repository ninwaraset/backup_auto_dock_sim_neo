import pandas as pd
import numpy as np

import matplotlib.pyplot as plt
import math
from os.path import expanduser as os
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn import metrics
def plot_check_cluster(i_number):
    zero = "0"
    if len(str(i_number)) >1:
        zero = ""

    df = pd.read_excel(os('~/neobotix_workspace/src/data_real_lidar_knn/data_'+str(zero)+str(i_number)+'.xls'))
    x_list = df['x'].tolist()
    y_list = df['y'].tolist()
    epc = df['end_point_cluster'].tolist()
    epc_list = []
    for i in range(len(epc)):
        if math.isnan(epc[i]) == False:
            epc_list.append(epc[i])
    print("epc_list : "+str(epc_list))
    color = ["b","g","r","c","m","y","k","w"]
    # plt.plot(x_list,y_list,(""))
    # plt.legend()
    # plt.show()
    point_1 = [0,0]
    point_2 = [0,0]
    point_3 = [0,0]
    er_tp = 0.004
    h_t = 0.11
    b_t = 0.25
    center_list = []

    

    spc = 0 
    color_num = 0
    checker = 0
    cusor = 0
    
    for i in range(len(epc_list)):
        length = int(epc_list[i]-spc)
        # print(length)
        for j in range(int(length)):
            
            cusor = int(spc+j)
            plt.plot(x_list[cusor],y_list[cusor],str(color[color_num])+"o")
            
            # print (cusor)
            if checker != cusor:
                print("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF")
            checker = cusor+1
            pass

        spc = epc_list[i]
        print("color_num : "+str(color_num))
        color_num += 1 
        if color_num == 8:
            color_num = 0
    print(center_list)
    for i in range(len(center_list)):
        plt.plot(x_list[center_list[i]],y_list[center_list[i]],"rd")
    plt.show()
    lidar_DBscan(x_list,y_list)

def lidar_DBscan(x_list,y_list):
    lidar_list = []
    for i in range(len(x_list)):

        lidar_list.append([x_list[i],y_list[i]])

    liadar_array = np.array(lidar_list)

    print(type(liadar_array))
    db = DBSCAN(eps=0.05, min_samples=2).fit(liadar_array)
    labels = db.labels_
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print("Estimated number of clusters: %d" % n_clusters_)
    print("Estimated number of noise points: %d" % n_noise_)
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

    plt.title(f"Estimated number of clusters: {n_clusters_}")
    plt.show()


def main(args=None):
    start_file = 1
    end_file = 50    
    lenght_file = end_file - start_file + 1
    current_file = start_file
    for i in range(lenght_file):
        print("-----------------------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------------------")
        print("current_file : " + str(current_file))
        plot_check_cluster(current_file)
        current_file += 1
    
# print()
if __name__ == '__main__':
    main()

    # plt.legend()
    # plt.show()