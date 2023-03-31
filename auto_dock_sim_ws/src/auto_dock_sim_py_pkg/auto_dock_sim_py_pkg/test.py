# import logging
# import threading
# import time

# def thread_function(name):
#     while(True):
#         logging.info("Thread %s: starting", name)
#         time.sleep(10)
#         logging.info("Thread %s: finishing", name)

# def two_function(name):
#     while(True):
#         logging.info("Thread %s: starting", name)
#         time.sleep(2)
#         logging.info("Thread %s: finishing", name)

# if __name__ == "__main__":
#     format = "%(asctime)s: %(message)s"
#     logging.basicConfig(format=format, level=logging.INFO,
#                         datefmt="%H:%M:%S")

#     logging.info("Main    : before creating thread")
#     x = threading.Thread(target=thread_function, args=(1,))
#     x.start()

#     y = threading.Thread(target=two_function, args=(2,))
#     y.start()


import control
import numpy as np
import matplotlib.pyplot as plt

# Define the plant transfer function
G = control.tf([1], [1, 1])

# Define the proportional gain
Kp = 1.0

# Define the proportional controller transfer function
C = control.tf([Kp], [1])

# Connect the controller to the plant
sys = control.feedback(C*G, 1)

# Define the time vector for simulation
t = np.linspace(0, 10, 1000)

# Define the desired setpoint
r = 1.0

# Define the initial condition
x0 = 0.0

# Simulate the closed-loop system response to a step input
t, y, x = control.step_response(sys, T=t, X0=x0, input=r)

# Plot the response
plt.plot(t, r*np.ones_like(t), 'r--', label='Setpoint')
plt.plot(t, y, label='Output')
plt.xlabel('Time (sec)')
plt.ylabel('Response')
plt.legend()
plt.show()


            