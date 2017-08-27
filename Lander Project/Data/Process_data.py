# -*- coding: utf-8 -*-
"""
Created on Tue Jun 27 19:31:13 2017

@author: jones
"""
import numpy as np
import matplotlib.pyplot as plt

plt.close("all")
results = np.loadtxt('./PID_tuning.txt')

fig, (ax1) = plt.subplots(1)
ax1.set_xlabel('time (hours)')
ax1.set_ylabel("altitude (km)")
ax1.grid()
ax1.plot(results[:, 0]/3600, results[:, 1]/1000,  label='altitude (km)')

ax2 = plt.twinx(ax1)
ax2.plot(results[:,0]/3600, results[:,2], "#FF7F0E", label = "angular velocity (m/s)")
ax2.set_ylabel("Velocity (m/s)")
plt.xlim(0)
plt.ylim(0)
plt.show()

#tu = 0.2