import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
    
    # read by default 1st sheet of an excel file
df = pd.read_excel('set_data_sim.xls')

# print(df)
# print(df["x"])
x_list_set = df['x'].tolist()
y_list_set = df['y'].tolist()

df2= pd.read_excel('tobe_set_data_sim.xls')

# print(df)
# print(df["x"])
x_list_2b = df2['x'].tolist()
y_list_2b = df2['y'].tolist()


se_t = [x_list_set ,y_list_set ]

alige = [x_list_2b,y_list_2b]


def icp(A, B, max_iterations=50, tolerance=1e-10):
    # A and B are the point sets to be aligned
    # max_iterations is the maximum number of iterations to perform
    # tolerance is the convergence threshold

    # Make a copy of B so we can update it
    B_copy = np.copy(B)
    # Initialize variables to store the transformation and error
    transformation = np.eye(3)
    error = float("inf")

    for i in range(max_iterations):
        # Find the nearest neighbors in B for each point in A
        distances = np.sum((A[:, None, :] - B_copy[None, :, :]) ** 2, axis=-1)
        indices = np.argmin(distances, axis=1)
        nearest_neighbors = B_copy[indices]

        # Compute the centroids of A and the nearest neighbors in B
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(nearest_neighbors, axis=0)

        # Subtract the centroids from the point sets
        A_centroid = A - centroid_A
        B_centroid = nearest_neighbors - centroid_B

        # Compute the covariance matrix
        covariance = np.sum(A_centroid * B_centroid, axis=0)

        # Compute the singular value decomposition of the covariance matrix
        U, S, V = np.linalg.svd(covariance)

        # Compute the rotation matrix
        R = np.dot(U, V)

        # Compute the translation vector
        t = centroid_B - np.dot(centroid_A, R)

        # Update the points in B_copy using the transformation
        B_copy = np.dot(B_copy, R) + t

        # Compute the new error
        error = np.sum((A - B_copy) ** 2)

        # Check for convergence
        if error < tolerance:
            break

    # Return the final transformation and error
    return transformation, error

# Generate some random points
A = np.random.rand(10, 2)
centroid_A = np.mean(A, axis=0)
plt.plot(A,"ro")
plt.plot(centroid_A,"bx")
plt.show()
print(centroid_A)
print("++++")
print(A.reshape(10, 2))

print(A[0].shape)
print(A[1])

B = np.random.rand(10, 2)
# print(B)
# Apply the ICP algorithm
transformation, error = icp(A, B)

# Print the results
print("Transformation matrix:")
print(transformation)
print("Error:", error)
