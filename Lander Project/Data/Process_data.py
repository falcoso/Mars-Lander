# -*- coding: utf-8 -*-
"""
Created on Tue Jun 27 19:31:13 2017

@author: jones
"""
import numpy as np
import matplotlib.pyplot as plt

plt.close("all")
results = np.loadtxt('./Autopilot scenario 5 soft.txt')


fig, (ax1) = plt.subplots(1)
ax1.set_xlabel('Altitude (km)')
ax1.set_ylabel("Error")
ax1.grid()
ax1.plot(results[:, 0]*1E-3, -results[:, 2],  label='Error')
plt.xlim(0,200)
plt.ylim(-500,4000)
#plt.yticks([0,2000,4000,6000,8000,10000])
#plt.ylim(0,1600)

ax2 = plt.twinx(ax1)
ax2.plot(0,0, label = 'Error')
ax2.plot(results[:,0]*1E-3, -results[:,1], "#FF7F0E", label = "Climb Speed")
ax2.set_ylabel("Speed (m/s)")
#plt.yticks([0,200,400,600,800,1000])
#plt.xlim(0,10000)
plt.ylim(ymax = 1000)
plt.xlim(0,200)
#ax2.set_yticks([0,10,20,30,40,50,60,70,80])
plt.yticks([-125,0,125,250,375,500,625,750,875,1000])
plt.legend(loc = 'upper right')
plt.show()

plt.tight_layout()
##tu = 0.2