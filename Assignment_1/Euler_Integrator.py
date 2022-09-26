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
x_list = []
v_list = []

# Euler integration
for t in range(len(t_array)):
    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate acceleration
    a = -k * x / m

    #update position & velocity
    x = x + dt * v
    v = v + dt * a


# convert trajectory lists into arrays, so they can be sliced
x_array = np.array(x_list)
v_array = np.array(v_list)


# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.title('Euler Integrator - Assignment1')
plt.plot(t_array, x_list, label='x (m)')
plt.plot(t_array, v_list, label='v (m/s)')
plt.legend()
plt.show()

# if dt is greater than ~0.01, then the result of Euler integration diverges from the true value

