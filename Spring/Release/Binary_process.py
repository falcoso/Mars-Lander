# -*- coding: utf-8 -*-
"""
Created on Sat Jul 15 14:30:25 2017

@author: jones
"""

import numpy as np

time = np.fromfile("time_array.bin", dtype = np.int64)

for i in time:
    print(i)