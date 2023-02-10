import numpy as np
import open3d as o3d

# Parameters:
initial_T = np.identity(4) # Initial transformation for ICP

distance = 0.1 # The threshold distance used for searching correspondences
# (closest points between clouds). I'm setting it to 10 cm.

# Read your point clouds:
source = o3d.io.read_point_cloud("point_cloud_1.xyz") 
target = o3d.io.read_point_cloud("point_cloud_0.xyz")

# Define the type of registration:
type = o3d.pipelines.registration.TransformationEstimationPointToPoint(False)
# "False" means rigid transformation, scale = 1

# Define the number of iterations (I'll use 100):
iterations = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 100)

# Do the registration:
result = o3d.pipelines.registration.registration_icp(source, target, distance, initial_T, type, iterations)