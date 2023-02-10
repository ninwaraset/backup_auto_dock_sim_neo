
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt
import math
from os.path import expanduser as os
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn import metrics

df = pd.read_excel(os('~/neobotix_workspace/src/data_real_lidar_knn/data_09.xls'))
x_list = df['x'].tolist()
y_list = df['y'].tolist()
lidar_list = []
for i in range(len(x_list)):

    lidar_list.append([x_list[i],y_list[i]])
    # print(lidar_list)


# plt.plot(x_list,y_list,"r.")

from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler

# centers = [[1, 1], [-1, 1], [1, -1]]
# X, labels_true = make_blobs(
#     n_samples=750, centers=centers, cluster_std=0.4, random_state=0
# )

liadar_array = np.array(lidar_list)
# print(X)


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