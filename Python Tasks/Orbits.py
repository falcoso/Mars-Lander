# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 22:41:16 2017

@author: jones
"""
import numpy as np
import matplotlib.pyplot as plt

#define gravitational constants, mass of planet and mass of pody
G = 6.67E-11
M = 6.42E23
m = 1

#initial conditions 
r0 = 17E3
r = np.array([r0,0,0])
v = np.array([0,1.1*np.sqrt(G*M/r0),0])

# simulation time, timestep and time
t_max = 20*np.sqrt(4*(np.pi**2)*(r0**3)/(G*M))
dt = 0.0001
t_array = np.arange(0, t_max, dt)

#create unit modulus function
def scal(r):      
    return np.sqrt(np.dot(r, r))

# r(t-dt) and r(t+dt)
rdt = r
r1 = np.zeros(len(r))

# initialise empty lists to record trajectories
r_list = []
v_list = []

#add initial states to list
r_list.append(r)
v_list.append(v)

#calculate position and velocity
a = -(G*M/(r0**3))*r
r = r + dt*v
v = v + dt*a

# verlot integration
for i in range(len(t_array)-1):
    # append current state to trajectories
    r_list.append(r)
    #v_list.append(v)
    
    #calculate new position and velocity using update equation
    a = -(G*M/(scal(r)**3))*r
    r1 = 2*r - rdt + dt*dt*a
    #v = (1/dt)*(r1-r)
    
    #move variables along one
    rdt=r
    r = r1  
  
# convert trajectory lists into arrays, so they can be indered more easily
r_array = np.array(r_list)

x_list = []
y_list = []
for i in r_array:
    x_list.append(i[0])
    y_list.append(i[1])
    
x_array = np.array(x_list)
y_array = np.array(y_list)    
#v_array = np.array(v_list)

plt.figure(1)
plt.clf()
plt.grid()
plt.plot(x_array, y_array, label='r (m)')
plt.plot(0,0, marker = 'o')
plt.show()

