import numpy as np
import matplotlib.pyplot as plt
# initialisation
m = 1
k = 1
x = 0
v = 1
t_max = 1000
dt = 1.95
t_array = np.arange(0, t_max, dt)
x_list = [x, ]
v_list = [v, ]
x_listV = np.zeros(len(t_array))
v_listV = np.zeros(len(t_array))

# for i in range(len(t_array)-1):
#     a = - k * x / m
#     x
for i in range(len(t_array)):

    a = -k * x_listV[i - 1] / m

    if i == 0:
        x_listV[i] = 0
        v_listV[i] = 1

    elif i == 1:
        # Euler Cromer Method for first iteration
        x_listV[i] = x_listV[i - 1] + dt * v_listV[i - 1]
        v_listV[i] = v_listV[i - 1] + dt * a

    else:
        x_listV[i] = 2 * x_listV[i - 1] - x_listV[i - 2] + dt ** 2 * a
        v_listV[i] = 1 / dt * (x_listV[i] - x_listV[i - 1])

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.title('Verlet Integrator - Assignment1')
plt.plot(t_array, x_listV, label='x (m)')
plt.plot(t_array, v_listV, label='v (m/s)')
plt.legend()
plt.show()

# These results are consistent with my analytical solution
# $\frac{d^2x}{dt^2} = -\frac{kx}{m}$
# $x = A\cos(\frac{kt}{m}) + B\sin(\frac{kt}{m})$
# $x(0) = 0 :. A = 0$
# $\frac{dx}{dt} (0) =0 :. B = \frac{m}{k}$
# $x = \frac{m}{k}\sin(\frac{kt}{m})$
# $v = \cos(\frac{kt}{m})$
# Both the displacement and velocity are sinusoidal, which both graphs display.
# For t_max =1000, the critical dt for which the Verlet integrator is no longer stable is 1.95,
# as the max and minimum points of oscillation are no longer constant.
# The Verlet integrator diverges significantly from the true values beyond dt = 1.99999.