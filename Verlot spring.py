# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 20:17:46 2017

@author: jones
"""

import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100
dt = 0.001
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []

# x(t-dt) and x(t+dt)
xdt = x
x1 = 0

# apply first step as euler
# append current state to trajectories
x_list.append(x)
v_list.append(v)

# calculate new position and velocity
a = -k * x / m
x = x + dt * v
v = v + dt * a

# verlot integration
for i in range(len(t_array)-1):
    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)
    
    #calculate new position and velocity using update equation
    a = -k*x/m
    x1 = 2*x - xdt + dt*dt*a
    v = (1/dt)*(x1-x)
    
    #move variables along one
    xdt=x
    x = x1  
  
# convert trajectory lists into arrays, so they can be indexed more easily
x_array = np.array(x_list)
v_array = np.array(v_list)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()
