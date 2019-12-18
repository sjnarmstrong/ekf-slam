import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import transforms3d as tr
from ConversionTools import *

savedir="./gt_"
postfix="_rpy.pdf"
#trajdir = "../Datasets/rgbd_dataset_freiburg1_xyz/groundtruth.txt"
trajdir = "../Datasets/rgbd_dataset_freiburg1_rpy/groundtruth.txt"
#trajdir = "../Datasets/rgbd_dataset_freiburg1_desk/groundtruth.txt"
traj=read_trajectory(trajdir)

X= np.array([traj[t][0,3] for t in traj])
Y= np.array([traj[t][1,3] for t in traj])
Z= np.array([traj[t][2,3] for t in traj])

"""
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot3D(X,Y,Z)
plt.show()
"""



"""
ax = plt.subplot(111, projection='polar')
ax.plot(theta, r)
ax.set_rmax(2)
ax.set_rticks([0.5, 1, 1.5, 2])  # less radial ticks
ax.set_rlabel_position(-22.5)  # get radial labels away from plotted line
ax.grid(True)

ax.set_title("A line plot on a polar axis", va='bottom')
plt.show()
"""
t0=list(traj.keys())[0]
times = np.array([t-t0 for t in traj])
angles = np.array([tr.euler.mat2euler(traj[t][:3, :3]) for t in traj])

angles = ((angles+np.pi)%(2*np.pi)-np.pi)*180/np.pi


dX = X[1:]-X[:-1]
dY = Y[1:]-Y[:-1]
dZ = Z[1:]-Z[:-1]
dRPY = angles[1:]-angles[:-1]
dT = times[1:]-times[:-1]

vx = dX/dT
vy = dY/dT
vz = dZ/dT
vRPY = (dRPY/dT[:,None]+np.pi)%(2*np.pi)-np.pi

max_x = np.max(np.abs(vx))
max_y = np.max(np.abs(vy))
max_z = np.max(np.abs(vz))
maxryp = np.max(np.abs(vRPY))
print("Max x:"+str(max_x))
print("Max y:"+str(max_y))
print("Max z:"+str(max_z))
print("Max ryp:"+str(maxryp))


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Plot of the Robot's Trajectory")
ax.plot3D(X,Y,Z)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
plt.show()

fig = plt.figure()
ax = plt.subplot(111)
ax.plot(times, angles[:, 0])
ax.set_yticks([30*i-180 for i in range(13)])
ax.grid(True)

ax.set_title("Plot of Roll vs. Time", va='bottom')
ax.set_ylabel(r"Roll ($^\circ$)")
ax.set_xlabel(r"Time (s)")
plt.show()
fig.savefig(savedir + "roll"+postfix, dpi=500, bbox_inches="tight", pad_inches=0)

fig = plt.figure()
ax = plt.subplot(111)
ax.plot(times, angles[:, 1])
ax.set_yticks([30*i-180 for i in range(13)])
ax.grid(True)

ax.set_title("Plot of Pitch vs. Time", va='bottom')
ax.set_ylabel(r"Pitch ($^\circ$)")
ax.set_xlabel(r"Time (s)")
plt.show()
fig.savefig(savedir + "pitch"+postfix, dpi=500, bbox_inches="tight", pad_inches=0)

fig = plt.figure()
ax = plt.subplot(111)
ax.plot(times, angles[:, 2])
ax.set_yticks([30*i-180 for i in range(13)])
ax.grid(True)

ax.set_title("Plot of Yaw vs. Time", va='bottom')
ax.set_ylabel(r"Yaw ($^\circ$)")
ax.set_xlabel(r"Time (s)")
plt.show()
fig.savefig(savedir + "yaw"+postfix, dpi=500, bbox_inches="tight", pad_inches=0)