import numpy as np
import sys
import transforms3d as tr

"""-----------------------------Code below taken from rgbd dataset tools---------------------------------------------"""

_EPS = np.finfo(float).eps * 4.0

def transform44(l):
    """
    Generate a 4x4 homogeneous transformation matrix from a 3D point and unit quaternion.

    Input:
    l -- tuple consisting of (stamp,tx,ty,tz,qx,qy,qz,qw) where
         (tx,ty,tz) is the 3D position and (qx,qy,qz,qw) is the unit quaternion.

    Output:
    matrix -- 4x4 homogeneous transformation matrix
    """
    t = l[1:4]
    q = np.array(l[4:8], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.array((
            (1.0, 0.0, 0.0, t[0]),
            (0.0, 1.0, 0.0, t[1]),
            (0.0, 0.0, 1.0, t[2]),
            (0.0, 0.0, 0.0, 1.0)
        ), dtype=np.float64)
    q *= np.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], t[0]),
        (q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], t[1]),
        (q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], t[2]),
        (0.0, 0.0, 0.0, 1.0)
    ), dtype=np.float64)


def read_trajectory(filename, matrix=True):
    """
    Read a trajectory from a text file.

    Input:
    filename -- file to be read
    matrix -- convert poses to 4x4 matrices

    Output:
    dictionary of stamped 3D poses
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    list = [[float(v.strip()) for v in line.split(" ") if v.strip() != ""] for line in lines if
            len(line) > 0 and line[0] != "#"]
    list_ok = []
    for i, l in enumerate(list):
        if l[4:8] == [0, 0, 0, 0]:
            continue
        isnan = False
        for v in l:
            if np.isnan(v):
                isnan = True
                break
        if isnan:
            sys.stderr.write("Warning: line %d of file '%s' has NaNs, skipping line\n" % (i, filename))
            continue
        list_ok.append(l)
    if matrix:
        traj = dict([(l[0], transform44(l[0:])) for l in list_ok])
    else:
        traj = dict([(l[0], l[1:8]) for l in list_ok])
    return traj
"""-----------------------------Code above taken from rgbd dataset tools---------------------------------------------"""

"""
Note 
X Roll   -a
Y Pitch  -b
Z yaw    -y

"""
def getXYZABYTLists(trajdir, ABS_TIME_TO_ZERO=None):
    traj = read_trajectory(trajdir)
    absTimes = np.array(list(traj.keys()))
    t0 = absTimes[0]

    if ABS_TIME_TO_ZERO is None:
        ABS_TIME_TO_ZERO = t0
    ind_time_to_zero = np.argmin(np.abs(ABS_TIME_TO_ZERO-absTimes))
    print("Zeroing index: "+str(ind_time_to_zero))
    invTraj=np.linalg.inv(traj[absTimes[ind_time_to_zero]])
    for timestamp in traj:
        traj[timestamp] = np.dot(invTraj,traj[timestamp])

    X = np.array([traj[t][0, 3] for t in traj])
    Y = np.array([traj[t][1, 3] for t in traj])
    Z = np.array([traj[t][2, 3] for t in traj])

    times = np.array([t - t0 for t in traj])
    angles = np.array([tr.euler.mat2euler(traj[t][:3, :3]) for t in traj])

    angles = ((angles + np.pi) % (2 * np.pi) - np.pi) * 180 / np.pi
    return X, Y, Z, angles[:, 0], angles[:, 1], angles[:, 2], times, absTimes