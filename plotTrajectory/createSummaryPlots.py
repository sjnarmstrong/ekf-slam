import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from ConversionTools import *

ground_truth_dirs={"XYZ":"../Results/GROUND_TRUTHS/rgbd_dataset_freiburg1_xyz-groundtruth.txt",
                   "RPY":"../Results/GROUND_TRUTHS/rgbd_dataset_freiburg1_rpy-groundtruth.txt",
                   "desk":"../Results/GROUND_TRUTHS/rgbd_dataset_freiburg1_desk-groundtruth.txt"}

Algs = ["EKF", "RGBD", "ORB"]
base_path = "../Results/"
datasetName = "XYZ"
baseFileName = "/trajectory_estimate.txt"
savedir = base_path+"SUMMARY/"+datasetName + "/"





"""
trajdir = base_path + "RGBD" + "/" + datasetName + baseFileName
X, Y, Z, roll, pitch, yaw, times, absTimes = getXYZABYTLists(trajdir)
fig = plt.figure()
ax = plt.subplot(111)
ax.plot(gttimes, gtroll)
ax.plot(gttimes, gtpitch)
ax.plot(gttimes, gtyaw)
ax.plot(times, roll)
#ax.set_yticks([30*i-180 for i in range(13)])
ax.grid(True)

ax.set_title("Plot of Absolute Pitch Error vs. Time", va='bottom')
ax.set_ylabel(r"Pitch ($^\circ$)")
ax.set_xlabel(r"Time (s)")
plt.show()
"""

all_errors = {}
for algName in Algs:
    trajdir = base_path + algName + "/" + datasetName + baseFileName
    X, Y, Z, roll, pitch, yaw, times, absTimes = getXYZABYTLists(trajdir)
    gtX, gtY, gtZ, gtroll, gtpitch, gtyaw, gttimes, gtAbsTimes = getXYZABYTLists(ground_truth_dirs[datasetName], absTimes[0])
    eX, eY, eZ, eR, eP, eYaw = [],[],[],[],[],[]
    for i,atme in enumerate(absTimes):
        minTdiffi = np.argmin(np.abs(gtAbsTimes-atme))
        eX.append(X[i] - gtX[minTdiffi])
        eY.append(Y[i] - gtY[minTdiffi])
        eZ.append(Z[i] - gtZ[minTdiffi])
        eR.append((roll[i] - gtroll[minTdiffi]+180) % 360-180)
        eP.append((pitch[i] - gtpitch[minTdiffi]+180) % 360-180)
        eYaw.append((yaw[i] - gtyaw[minTdiffi]+180) % 360-180)
        #print("timediff: "+str(gtAbsTimes[minTdiffi]-atme))
    all_errors[algName] = {"X":X,"Y":Y,"Z":Z,"eX":eX,"eY":eY,"eZ":eZ,"eR":eR, "eP":eP, "eYaw":eYaw, "times":times}
    rmserror = np.sqrt((np.array(eX)**2+np.array(eY)**2+np.array(eZ)**2).mean()/3.0)
    rot_rmserror = np.sqrt((np.array(eR)**2+np.array(eP)**2+np.array(eYaw)**2).mean()/3.0)
    print(datasetName+algName+"   RMSE: "+str(rmserror*100))
    print(algName+"   rRMSE: "+str(rot_rmserror))
    rmserror = np.sqrt((np.array(eX) ** 2 + np.array(eY) ** 2 + np.array(eZ) ** 2).mean())
    rot_rmserror = np.sqrt((np.array(eR) ** 2 + np.array(eP) ** 2 + np.array(eYaw) ** 2).mean())
    print(algName + "   RMSE2: " + str(rmserror * 100))
    print(algName + "   rRMSE2: " + str(rot_rmserror))
"""
for algName in Algs:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Plot of the Robot's Trajectory")
    ax.plot3D(all_errors[algName]['X'],all_errors[algName]['Y'],all_errors[algName]['Z'])
    ax.plot3D(gtX,gtY,gtZ)
    ax.set_xlabel('error x (m)')
    ax.set_ylabel('error y (m)')
    ax.set_zlabel('error z (m)')
    fig.savefig(savedir + "errtraj_orth.pdf", dpi=500, bbox_inches="tight", pad_inches=0.1)
    plt.show()

for algName in Algs:
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Plot of the Robot's Trajectory")
ax.plot3D(eX,eY,eZ)
ax.set_xlabel('error x (m)')
ax.set_ylabel('error y (m)')
ax.set_zlabel('error z (m)')
fig.savefig(savedir + "errtraj_orth.pdf", dpi=500, bbox_inches="tight", pad_inches=0.1)
#plt.show()
ax.view_init(0,-90)
fig.savefig(savedir + "errtraj_xz.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
ax.view_init(0,0)
fig.savefig(savedir + "errtraj_yz.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
ax.view_init(90,0)
fig.savefig(savedir + "errtraj_xy.pdf", dpi=500, bbox_inches="tight", pad_inches=0)

"""

fig = plt.figure()
ax = plt.subplot(111)
for algName in Algs:
    ax.plot(all_errors[algName]['times'], all_errors[algName]['eYaw'], label=algName)
#ax.set_yticks([30*i-180 for i in range(13)])
ax.grid(True)
ax.legend()

ax.set_title("Plot of Absolute Yaw Error vs. Time", va='bottom')
ax.set_ylabel(r"Yaw ($^\circ$)")
ax.set_xlabel(r"Time (s)")
fig.savefig(savedir + "erryaw.pdf", dpi=500, bbox_inches="tight", pad_inches=0)

fig = plt.figure()
ax = plt.subplot(111)
for algName in Algs:
    ax.plot(all_errors[algName]['times'], all_errors[algName]['eP'], label=algName)
#ax.set_yticks([30*i-180 for i in range(13)])
ax.grid(True)
ax.legend()

ax.set_title("Plot of Absolute Pitch Error vs. Time", va='bottom')
ax.set_ylabel(r"Pitch ($^\circ$)")
ax.set_xlabel(r"Time (s)")
fig.savefig(savedir + "errpitch.pdf", dpi=500, bbox_inches="tight", pad_inches=0)


fig = plt.figure()
ax = plt.subplot(111)
for algName in Algs:
    ax.plot(all_errors[algName]['times'], all_errors[algName]['eR'], label=algName)
#ax.set_yticks([30*i-180 for i in range(13)])
ax.grid(True)
ax.legend()

ax.set_title("Plot of Absolute Roll Error vs. Time", va='bottom')
ax.set_ylabel(r"Roll ($^\circ$)")
ax.set_xlabel(r"Time (s)")
fig.savefig(savedir + "errroll.pdf", dpi=500, bbox_inches="tight", pad_inches=0)


fig = plt.figure()
ax = plt.subplot(111)
for algName in Algs:

    ax.plot(all_errors[algName]['times'], np.sqrt(np.array(all_errors[algName]['eX'])**2+
                           np.array(all_errors[algName]['eY'])**2+
                           np.array(all_errors[algName]['eZ'])**2), label=algName)
    #ax.set_yticks([30*i-180 for i in range(13)])
ax.grid(True)
ax.legend()
ax.set_title("Plot of Absolute Displacement Error vs. Time", va='bottom')
ax.set_ylabel(r"Displacement ($m$)")
ax.set_xlabel(r"Time (s)")
fig.savefig(savedir + "errdisplacement.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
#plt.show()