import numpy as np
import os
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
plt.ioff()
for timenr,f_name in enumerate(os.listdir('outputVar')):

    Variances = np.genfromtxt('outputVar/'+f_name, delimiter=',')
    Means = np.genfromtxt('outputMean/'+f_name, delimiter=',')
    covs = []
    for i in range(int(len(Variances)/3)):
        ind = int(i*3)
        cov = Variances[ind:ind+2, ind:ind+2]
        mean = Means[ind:ind+2]

    #code taken form stack overvlow
        lambda_, v = np.linalg.eig(cov)
        lambda_ = np.sqrt(lambda_)

        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        for j in range(1, 4):
            ell = Ellipse(xy=(mean[0], mean[1]),
                          width=lambda_[0] * j * 2, height=lambda_[1] * j * 2,
                          angle=np.rad2deg(np.arccos(v[0, 0])))
            ell.set_facecolor('none')
            ell.set_edgecolor('black')
            ax.add_artist(ell)
        ax.set_ylim(mean[1] - ell.height, mean[1] + ell.height)
        ax.set_xlim(mean[0] - ell.width, mean[0] + ell.width)
        ax.set_title("Plot of landmarks certainty for t="+str(timenr), va='bottom')
        ax.set_ylabel(r"Y (m)")
        ax.set_xlabel(r"X (x)")
    #endcode
        os.makedirs("OutputCovPlots/"+str(i)+"/",exist_ok=True)
        fig.savefig("OutputCovPlots/"+str(i)+"/t_"+str(timenr)+".pdf", dpi=300, bbox_inches="tight", pad_inches=0)
        plt.close(fig)