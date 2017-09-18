# -*- coding: utf-8 -*-
"""
Created on Tue Jun 27 19:31:13 2017

@author: jones
"""
import numpy as np
import matplotlib.pyplot as plt

plt.close("all")
results = np.loadtxt('./Angle_plot.txt')

pout = np.array([j/np.sqrt(j**2+k**2) for i,j,k,l in results])

plt.plot(results[:,0], pout)
plt.ylim(pout[0],pout.max())

fig, (ax1) = plt.subplots(1)
ax1.set_xlabel('time (hours)')
ax1.set_ylabel("Attitude Angle (rads)")
ax1.grid()
ax1.plot(results[:, 0], results[:, 3],  label='attitude angle')
plt.ylim(ymax = -2.95)

ax2 = plt.twinx(ax1)
ax2.plot(results[:,0], results[:,1]*1e5, "#FF7F0E", label = "attitude angle")
#ax2.plot(results[:,0], results[:,2]*1E5, "#2ca02c", label = "attitude angle")
ax2.set_ylabel("Error 1E-5")
plt.ylim(1.83,1.85)
plt.xlim(results[:,0].min(),4100)
plt.show()

plt.tight_layout()
##tu = 0.2