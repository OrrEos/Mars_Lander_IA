import numpy as np
import matplotlib.pyplot as plt


def escape_velocity(r):
    v_mag = np.sqrt((2*G*M)/(np.linalg.norm(r)))
    return v_mag

# mass, spring constant, initial position and velocity
m = 1.0
G = 6.6743E-11
M = 6.42E23
R_mars = 3.3895E6  # radius of Mars in metres

# Initial Conditions ========================================
r = np.array([0, 0, 4E6])
vel_esc = escape_velocity(r)
r_mag = np.linalg.norm(r)
v = np.array([2* np.sqrt(G * M / R_mars), 0, 0])
'''straight down descent = set r_z to any height greater than R_Mars
    if v_x = k * np.sqrt(G * M / R_mars), 
    circular orbit: k = 1
    elliptical orbit: 1<k<2
    hyperbolic escape: k>2'''

# ==========================================================

r_list = [r, ]
v_list = [v, ]
r_1 = r

# Kepler's Law gives an equation for time period for elliptical orbit
T = np.sqrt(4 * (np.pi ** 2) * (r_mag ** 3) / (G * M))
t_max = 3*T
dt = 0.1
t_array = np.arange(0, t_max, dt)

# initial integration
r_mag = np.linalg.norm(r)
a = -(G*M*r)/(r_mag**3)
r = r + dt*v
v = v + dt*a
# v = 1/dt * (r - r_1)


for i in range(len(t_array)-1):
    r_list.append(r)
    v_list.append(v)
    r_mag = np.linalg.norm(r)
    a = -(G*M*r) / (r_mag**3)
    r1 = 2*r - r_1 + (dt**2)*a
    v = 1/(2*dt) * (r1 - r_1)
    # update previous & current variables
    r_1 = r
    r = r1

r_array = np.array(r_list)
v_array = np.array(v_list)

fig = plt.figure(111, figsize=(4, 4))
# ax = fig.add_subplot(211)
# # ax.plot(t_array, r_array[:, 2] - R_mars)
# ax.set_title('Altitude of Orbit (for straight-down descent)')
# ax.set_xlabel('Time (s)')
# ax.set_ylabel('R_z (m)')

ax = fig.add_subplot(111)
ax.plot(r_array[:,0], r_array[:, 2])
ax.plot(0,0, marker = 'o')
ax.set_title('Plot in orbital plane')
ax.axis('equal')
# xy_mins = np.array([min(r_array[:, 1]), min(r_array[:, 2])])
# xy_maxs = np.array([max(r_array[:, 1]), max(r_array[:, 2])])
# xy_min = min(xy_mins)
# xy_max = max(xy_maxs)
# ax.set_xlim(xy_min, xy_max)
# ax.set_ylim(xy_min, xy_max)
plt.show()

# plt.figure(1)
# plt.clf()
# plt.grid()
# plt.plot(r_array[:, 0], r_array[:, 2], label='r (m)')
# plt.plot(0, 0, marker='o')
# plt.show()