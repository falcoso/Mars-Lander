# -*- coding: utf-8 -*-
"""
Created on Tue Jun 27 19:31:13 2017

@author: jones
"""

import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt('trajectories.txt')
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='x (m)')
plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
plt.legend()
plt.show()