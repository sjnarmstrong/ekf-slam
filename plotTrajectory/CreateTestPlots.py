import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from ConversionTools import *

ground_truth_dirs={"XYZ":"../Datasets/rgbd_dataset_freiburg1_xyz/groundtruth.txt",
                   "RPY":"../Datasets/rgbd_dataset_freiburg1_rpy/groundtruth.txt",
                   "desk":"../Datasets/rgbd_dataset_freiburg1_desk/groundtruth.txt"}
base_path = "../Results/"
#algName = "ORB"
#algName = "RGBD"
algName = "EKF"
#Datasets = ["XYZ", "RPY", "desk"]
Datasets = ["XYZ"]
baseFileName = "/trajectory_estimate.txt"
#baseFileName = "/trajectory_estimate_test_2.txt"
for datasetName in Datasets:
    trajdir = base_path+algName+"/"+datasetName+baseFileName
    savedir = base_path+algName+"/"+datasetName + "/"

    X,Y,Z,roll,pitch,yaw,times,absTimes = getXYZABYTLists(trajdir)
    gtX,gtY,gtZ,gtroll,gtpitch,gtyaw,gttimes,gtAbsTimes = getXYZABYTLists(ground_truth_dirs[datasetName], absTimes[0])


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Plot of the Robot's Trajectory")
    ax.plot3D(X,Y,Z)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    fig.savefig(savedir + "traj_orth.pdf", dpi=500, bbox_inches="tight", pad_inches=0.1)
    #plt.show()
    ax.view_init(0,-90)
    fig.savefig(savedir + "traj_xz.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
    ax.view_init(0,0)
    fig.savefig(savedir + "traj_yz.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
    ax.view_init(90,0)
    fig.savefig(savedir + "traj_xy.pdf", dpi=500, bbox_inches="tight", pad_inches=0)

    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(times, roll)
    ax.set_yticks([30*i-180 for i in range(13)])
    ax.grid(True)

    ax.set_title("Plot of Roll vs. Time", va='bottom')
    ax.set_ylabel(r"Roll ($^\circ$)")
    ax.set_xlabel(r"Time (s)")
    fig.savefig(savedir + "roll.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
    #plt.show()

    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(times, pitch)
    ax.set_yticks([30*i-180 for i in range(13)])
    ax.grid(True)

    ax.set_title("Plot of Pitch vs. Time", va='bottom')
    ax.set_ylabel(r"Pitch ($^\circ$)")
    ax.set_xlabel(r"Time (s)")
    fig.savefig(savedir + "pitch.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
    #plt.show()

    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(times, yaw)
    ax.set_yticks([30*i-180 for i in range(13)])
    ax.grid(True)

    ax.set_title("Plot of Yaw vs. Time", va='bottom')
    ax.set_ylabel(r"Yaw ($^\circ$)")
    ax.set_xlabel(r"Time (s)")
    fig.savefig(savedir + "yaw.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
    #plt.show()





    ######################GT ERROR
    eX, eY, eZ, eR, eP, eYaw = [],[],[],[],[],[]
    for i,atme in enumerate(absTimes):
        minTdiffi = np.argmin(np.abs(gtAbsTimes-atme))
        eX.append(X[i] - gtX[minTdiffi])
        eY.append(Y[i] - gtY[minTdiffi])
        eZ.append(Z[i] - gtZ[minTdiffi])
        eR.append(roll[i] - gtroll[minTdiffi])
        eP.append(pitch[i] - gtpitch[minTdiffi])
        eYaw.append(yaw[i] - gtyaw[minTdiffi])
        #print("timediff: "+str(gtAbsTimes[minTdiffi]-atme))
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



    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Plot of the Robot's Trajectory")
    ax.plot3D(X,Y,Z, label=algName)
    ax.plot3D(gtX,gtY,gtZ, label="Ground Truth")
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.legend()
    fig.savefig(savedir + "gttraj_orth.pdf", dpi=500, bbox_inches="tight", pad_inches=0.1)
    plt.show()
    ax.view_init(0,-90)
    fig.savefig(savedir + "gttraj_xz.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
    ax.view_init(0,0)
    fig.savefig(savedir + "gttraj_yz.pdf", dpi=500, bbox_inches="tight", pad_inches=0)
    ax.view_init(90,0)
    fig.savefig(savedir + "gttraj_xy.pdf", dpi=500, bbox_inches="tight", pad_inches=0)



    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(times, eYaw)
    #ax.set_yticks([30*i-180 for i in range(13)])
    ax.grid(True)

    ax.set_title("Plot of Absolute Yaw Error vs. Time", va='bottom')
    ax.set_ylabel(r"Yaw ($^\circ$)")
    ax.set_xlabel(r"Time (s)")
    fig.savefig(savedir + "erryaw.pdf", dpi=500, bbox_inches="tight", pad_inches=0)

    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(times, eP)
    #ax.set_yticks([30*i-180 for i in range(13)])
    ax.grid(True)

    ax.set_title("Plot of Absolute Pitch Error vs. Time", va='bottom')
    ax.set_ylabel(r"Pitch ($^\circ$)")
    ax.set_xlabel(r"Time (s)")
    fig.savefig(savedir + "errpitch.pdf", dpi=500, bbox_inches="tight", pad_inches=0)

    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(times, eR)
    #ax.set_yticks([30*i-180 for i in range(13)])
    ax.grid(True)

    ax.set_title("Plot of Absolute Roll Error vs. Time", va='bottom')
    ax.set_ylabel(r"Roll ($^\circ$)")
    ax.set_xlabel(r"Time (s)")
    fig.savefig(savedir + "errroll.pdf", dpi=500, bbox_inches="tight", pad_inches=0)


    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(times, np.sqrt(np.array(eX)**2+np.array(eY)**2+np.array(eZ)**2))
    #ax.set_yticks([30*i-180 for i in range(13)])
    ax.grid(True)

    ax.set_title("Plot of Absolute Displacement Error vs. Time", va='bottom')
    ax.set_ylabel(r"Displacement ($m$)")
    ax.set_xlabel(r"Time (s)")
    fig.savefig(savedir + "errdisplacement.pdf", dpi=500, bbox_inches="tight", pad_inches=0)