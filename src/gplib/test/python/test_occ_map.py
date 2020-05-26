import numpy as np
import math
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process.kernels import RBF
from sklearn.gaussian_process.kernels import Matern

def DistCalculation(pt1,pt2):
    dist_sq = (pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2
    dist = math.sqrt(dist_sq)
    return dist

# This file is used to test can GP represent certain occupancy grid map
x = np.arange(0.5,10.5,1.0)
y = np.arange(0.5,10.5,1.0)
training_X = [[idx1,idx2] for idx1 in x for idx2 in y]

obs1_x = np.arange(0.5,5.5,1.0)
obs1_y = np.arange(2.5,4.5,1.0)
obs1 = [[idx1,idx2] for idx1 in obs1_x for idx2 in obs1_y]

obs2_x = np.arange(6.5,8.5,1.0)
obs2_y = np.arange(0.5,4.5,1.0)
obs2 = [[idx1,idx2] for idx1 in obs2_x for idx2 in obs2_y]

obs3_x = np.arange(0.5,2.5,1.0)
obs3_y = np.arange(5.5,8.5,1.0)
obs3 = [[idx1,idx2] for idx1 in obs3_x for idx2 in obs3_y]

obs4_x = np.arange(5.5,7.5,1.0)
obs4_y = np.arange(6.5,10.5,1.0)
obs4 = [[idx1,idx2] for idx1 in obs4_x for idx2 in obs4_y]

obs_ = obs1 + obs2 + obs3 + obs4
print(training_X)

training_Y = []
for item in training_X:
    if item in obs_:
        training_Y.append(1)
    else:
        training_Y.append(0)


print(training_Y)
# Specify Gaussian Processes with fixed and optimized hyperparameters
gp_opt = GaussianProcessClassifier(kernel=1.0 * RBF(length_scale=1))
gp_opt.fit(training_X,training_Y)
print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))
# print("Log Marginal Likelihood (optimized): %.3f"
#       % gp_opt.log_marginal_likelihood(gp_opt.kernel_.theta))


fig = plt.figure()
ZZ = np.empty([10,10])
for idx1, row in enumerate(x):
    for idx2,col in enumerate(y):
        K = [row,col]
        p_occ = gp_opt.predict_proba(np.reshape(K,(-1,2)))[:,1]
        occ_ = gp_opt.predict(np.reshape(K,(-1,2)))
        ZZ[idx1,idx2] = p_occ
        plt.text(row,col,"{}".format(round(p_occ[0],3)), color='black',fontsize=10)
        if occ_ == 1:
            plt.text(row + 0.25, col+0.25, "OCC", color='black', fontsize=10)
        else:
            plt.text(row + 0.25, col + 0.25, "FREE", color='black', fontsize=10)

XX ,YY = np.mgrid[0:10:11j,0:10:11j]

CMAP = plt.get_cmap('jet')
plt.pcolormesh(XX,YY,ZZ,cmap=CMAP)
cb = plt.colorbar(shrink = 1.0)

plt.show()

