import numpy as np
import math
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process.kernels import RBF
from sklearn.gaussian_process.kernels import Matern


# This file is used to test can GP represent certain occupancy grid map with given sensors position and measurements
x = np.arange(0.5,10.5,1.0)
y = np.arange(0.5,10.5,1.0)

training_X = [[0.5,6.5],[1.5,7.5],[1.5,6.5],[1.5,5.5],[2.5,6.5],[2.5,4.5],[3.5,5.5],[3.5,4.5],[3.5,3.5],[4.5,4.5]]
training_Y = np.array([1,1,1,1,0,0,0,0,1,0])

# Specify Gaussian Processes with fixed and optimized hyperparameters
gp_opt = GaussianProcessClassifier(RBF(length_scale=1.0))
gp_opt.fit(training_X,training_Y)
print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))
print("===========================================")
lml,gradient = gp_opt.log_marginal_likelihood(theta=[0.44992595],eval_gradient=True)
print("The log likelihood is {}".format(lml))
print("The gradient is {}".format(gradient))
# print("Log Marginal Likelihood (optimized): %.3f"
#       % gp_opt.log_marginal_likelihood(gp_opt.kernel_.theta))


fig = plt.figure()
ZZ = np.empty([10,10])
for idx1, row in enumerate(x):
    for idx2,col in enumerate(y):
        K = [row,col]
        # print("({},{})".format(row,col))
        # print("({},{})".format(idx1,idx2))
        
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

