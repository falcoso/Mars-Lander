import numpy as np
import control as ct
import matplotlib.pyplot as plt

G = 6.673E-11
M = 6.42E23
m = 200
R0 = 3386000

Kp = 0.01
Ki = 1E-6
Kd = 0.1

h0 = 500

lander = ct.tf([1],[m, 0, -2*G*M/(R0+h0)**3])
fb = ct.tf([Kd, Kp, Ki],[1,0])
ct.bode_plot(lander)
# lander = lander/(1-lander*fb)
ct.bode_plot(lander*fb)
plt.figure()
ct.nyquist_plot(lander*fb)
T, y = ct.impulse_response(ct.feedback(lander*fb,))
plt.figure()
plt.plot(T,y)
plt.show()