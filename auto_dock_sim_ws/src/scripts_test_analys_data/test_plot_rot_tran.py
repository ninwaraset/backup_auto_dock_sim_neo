import numpy as np
import matplotlib.pyplot as plt
import math 


theta = -math.pi*2/3
tran_x = 1
tran_y = 0


point_x_1 = 0
point_y_1 = 0

point_x_2 = 1
point_y_2 = 0

# line_x = [point_x_1,point_x_2]
# line_y = [point_y_1,point_y_2]
line_x = [0,tran_x]
line_y = [0,tran_y]

point_x_1_bar = (point_x_1*math.cos(theta) + point_y_1*math.sin(theta)) + tran_x
point_y_1_bar = (point_x_1*math.sin(theta) + point_y_1*math.cos(theta)) + tran_y

# point_x_1_bar = (point_x_1) + tran_x
# point_y_1_bar = (point_x_1) + tran_y

# point_x_1_bar = (point_x_2*math.cos(theta) + point_y_2*math.sin(theta)) + tran_x
# point_y_1_bar = (point_x_2*math.sin(theta) + point_y_2*math.cos(theta)) + tran_y



point_x_2_bar = (point_x_2*math.cos(theta) + point_y_2*math.sin(theta)) + tran_x
point_y_2_bar = (point_x_2*math.sin(theta) + point_y_2*math.cos(theta)) + tran_y

line_x_bar = [point_x_1_bar,point_x_2_bar]
line_y_bar = [point_y_1_bar,point_y_2_bar]

# line_x_bar = [point_x_1_bar,point_x_2]
# line_y_bar = [point_y_1_bar,point_y_2]



x_axis = 1.5*2
y_axis = 1.2*2



fig = plt.gcf()
ax = fig.gca()

circle1= plt.Circle((0,0), 1,ls = "-", color='#00ff00', fill=False)
ax.add_patch(circle1)
circle2= plt.Circle((tran_x,tran_y), 1, ls = "-.",color='#FFA500', fill=False)
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

plt.plot(point_x_1,point_y_1,marker="^",color="r")
plt.plot(point_x_2,point_y_2,marker="^",color="#4e0090")
plt.plot(line_x,line_y,ls = "-", color='r')



plt.plot(point_x_1_bar,point_y_1_bar,marker="*",color="b")
plt.plot(point_x_2_bar,point_y_2_bar,marker="*",color="g")
plt.plot(line_x_bar,line_y_bar,ls = "-", color='y')



plt.show()
# 