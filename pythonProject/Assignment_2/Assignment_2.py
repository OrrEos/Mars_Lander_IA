import numpy as np
import matplotlib.pyplot as plt

# Straight Down Descent

# mass, spring constant, initial position and velocity
m = 1.0
G = 6.6743 * 10 ** (-11)
M = 6.42 * 10 ** (23)
R_mars = 3389.5e3  # radius of mars in metres

# 3D vectors
r_x, r_y, r_z = 0.0, 0.0, R_mars + 100.0  # where r_z is initial distance from planet
v_x, v_y, v_z = 0.0, 0.0, 0.0  # initial velocity
r = np.array([r_x, r_y, r_z])
v = np.array([v_x, v_y, v_z])

# simulation time, timestep and time
t_max = 10
dt = 0.01
t_array = np.arange(0, t_max, dt)

# Euler-Cromer Method =======================

# initialise empty lists to record trajectories
r_list = []
v_list = []

# Euler integration
for t in range(len(t_array)):
    # append current state to trajectories
    r_list.append(r)
    v_list.append(v)

    # calculate acceleration
    pos_mag = np.linalg.norm(r)       # magnitude of position
    a = -(G * M * r) / (pos_mag ** 3)  # + ((np.cross(position,(m*velocity)))**2)/(pos_mag**4)

    # update position & velocity
    v = v + dt * a
    r = r + dt * v

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(r_list)
v_array = np.array(v_list)

# print(x_array[0], x_array[1])
# print(x_array[:,2])

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array[:, 2] - R_mars, label='Altitude')
plt.ylim(ymax=110, ymin=0)
plt.legend()
plt.show()

# Circular Orbit

# mass, spring constant, initial position and velocity
m = 1.0
G = 6.6743 * 10 ** (-11)
M = 6.42 * 10 ** (23)
R_mars = 3389.5e3  # radius of mars in metres


def escape_velocity(r):
    v_mag = np.linalg.norm(np.sqrt((2 * G * M * r) / ((np.linalg.norm(r)) ** 2)))
    return v_mag


# 3D vectors
r_x, r_y, r_z = 0.0, 0.0, R_mars + 100.0  # where r_z is initial distance from planet
r = np.array([r_x, r_y, r_z])

# escape velocity:
vel_esc = escape_velocity(r)
print(vel_esc)
# in order not to escape orbit, velocity can't exceed vel_esc

v_x, v_y, v_z = 7000, 0.0, 0.0  # initial velocity
v = np.array([v_x, v_y, v_z])

# simulation time, timestep and time
t_max = 100
dt = 0.1
t_array = np.arange(0, t_max, dt)

# Euler Cromer Method =======================

# initialise empty lists to record trajectories
r_list = []
v_list = []

rho = np.hypot(r[2], r[0])
phi = np.arctan2(r[2], r[0])
rho_list = []
phi_list = []

# Euler integration
for t in range(len(t_array)):
    # print(position, velocity, a)

    # append current state to trajectories
    r_list.append(r)
    v_list.append(v)

    # polar coordinates:
    rho = np.hypot(r[2], r[0])
    phi = np.arctan2(r[2], r[0])
    rho_list.append(rho)
    phi_list.append(np.rad2deg(phi))

    # calculate acceleration, now w/angular momentum
    pos_mag = np.linalg.norm(r)
    a = (-G * M * r) / (m * pos_mag ** 3) + ((np.cross(r, (m * v))) ** 2) / (pos_mag ** 4)

    # update position & velocity
    v = v + dt * a
    r = r + dt * v

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(r_list)
v_array = np.array(v_list)

# print(x_array[0], x_array[1])
# print(x_array[:,2])

# plot the position-time graph
fig, axs = plt.subplots(2)
axs[0].plot(t_array, x_array[:, 2] - R_mars, label='Altitude')

axs[1].plot(t_array, v_array[:, 0] - R_mars, label='Tangential Velocity')
plt.show()

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, )
ax.plot(phi_list, rho_list)
# ax.set_rmax(2)
# ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
# ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
ax.grid(True)

ax.set_title("Circular Orbit", va='bottom')
plt.show()

print(phi_list)









