# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# 1 =====================================================
# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 1000
dt = 1.95
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_listE = np.zeros(len(t_array))
v_listE = np.zeros(len(t_array))

for t in range(len(t_array)):
    if t == 0:
        x_listE[t] = 0
        v_listE[t] = 1
    else:
        # calculate new position and velocity
        a = -k * x_listE[t - 1] / m
        x_listE[t] = x_listE[t - 1] + dt * v_listE[t - 1]
        v_listE[t] = v_listE[t - 1] + dt * a

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_listE, label='x (m)')
plt.plot(t_array, v_listE, label='v (m/s)')
plt.legend()
plt.show()

# if dt is greater than ~0.01, then the result of Euler integration diverges from the true value

# Euler Cromer Method2
m = 1
k = 1
x = 0
v = 1
# initialise empty lists to record trajectories
x_list = []
v_list = []

# Euler integration
for t in range(len(t_array)):
    #print(position, velocity, a)
    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate acceleration
    a = -k * x / m

    #update position & velocity
    x = x + dt * v
    v = v + dt * a


# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)


# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_list, label='x (m)')
plt.plot(t_array, v_list, label='v (m/s)')
plt.legend()
plt.show()

#if dt is greater than ~0.01, then the result of Euler integration diverges from the true value

# ======== Verlet Integrator =======

x_listV = np.zeros(len(t_array))
v_listV = np.zeros(len(t_array))

for i in range(len(t_array)):

    a = -k * x_listV[i - 1] / m

    if i == 0:
        x_listV[i] = 0
        v_listV[i] = 1

    elif i == 1:
        # Euler Cromer Method for first iteration
        x_listV[i] = x_listV[i - 1] + dt * v_listV[i - 1]
        v_listV[t] = v_listV[i - 1] + dt * a

    else:
        x_listV[i] = 2 * x_listV[i - 1] - x_listV[i - 2] + dt ** 2 * a
        v_listV[i] = 1 / dt * (x_listV[i] - x_listV[i - 1])

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
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