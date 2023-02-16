import numpy as np
import matplotlib.pyplot as plt
import math 

# -0.04265704124809199, 1.2965369345324196

# distance ,theta(radian) :  [1.2972384691238403, -0.03288888931274392]

origin_x = 0
origin_y = 0

vertex_x = -0.04265704124809199
vertex_y = 1.2965369345324196

tran_x = vertex_x
tran_y = vertex_y

line_1_x = [origin_x,tran_x]
line_1_y = [origin_y,tran_y]


# theta = -math.pi*2/3
# dist = 1
theta = -1.3843246951247097
dist = 0.35

blue_x = dist*math.cos(theta) + tran_x
blue_y = dist*math.sin(theta) + tran_y


line_2_x = [tran_x,blue_x]
line_2_y = [tran_y,blue_y]

plt.plot(origin_x,origin_y,marker="o",color="g")
plt.plot(tran_x,tran_y,marker="^",color="r")
plt.plot(line_1_x,line_1_y,ls = "-", color='y')

plt.plot(blue_x,blue_y,marker="*",color="b")
plt.plot(line_2_x,line_2_y,ls = "-", color='m')

# plt.plot(point_x_2_bar,point_y_2_bar,marker="*",color="g")
# plt.plot(line_x_bar,line_y_bar,ls = "-", color='y')



x_axis = 1.5*2
y_axis = 1.2*2



fig = plt.gcf()
ax = fig.gca()

circle1= plt.Circle((0,0), 1,ls = "-", color='#00ff00', fill=False)
# ax.add_patch(circle1)
circle2= plt.Circle((tran_x,tran_y), dist, ls = "-.",color='#FFA500', fill=False)
ax.add_patch(circle2)



ax.spines['left'].set_position('zero')
ax.spines['bottom'].set_position('zero')


ax.spines['right'].set_color('c')
ax.spines['right'].set_linestyle("--")
ax.spines['right'].set_position(('data', tran_x))
ax.spines['top'].set_color('c')
ax.spines['top'].set_linestyle("-.")
ax.spines['top'].set_position(('data', tran_y))

ax.xaxis.set_ticks_position('bottom')
ax.yaxis.set_ticks_position('left')
# ax.xaxis.set_ticks_position('top')
# ax.yaxis.set_ticks_position('right')
plt.axis([-x_axis/2,x_axis, -y_axis/2, y_axis])




plt.show()
# 