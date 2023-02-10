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


se_t = [y_list_set ,x_list_set ]
print(se_t)
alige = [y_list_2b,x_list_2b]

centroid_A = np.mean(se_t, axis=1)
print(centroid_A)
centroid_B = np.mean(alige, axis=1)
print(centroid_B)

plt.ylim([-3, 3])
plt.xlim([-3, 3])
plt.plot(se_t[0],se_t[1],"ro")
plt.plot(alige[0],alige[1],"go")



plt.plot(centroid_A[0],centroid_A[1],"bx")
plt.plot(centroid_B[0],centroid_B[1],"yx")

print([1,2,3,4]-1)

# plt.show()
# print(centroid_A)

import math
from icp import icp
# import icp

if __name__ == '__main__':

    plt.legend()
    plt.show()