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

# Build the "continuous" map
row_coordinate = np.linspace(0,5,26)
col_coordinate = np.linspace(0,5,26)

# Generate training data
sensor1_center = [1.5,2.5]
sensor2_center = [3.5,1.5]
Rs = 1

pts1_sensing_range_ = []
for row in row_coordinate:
    for col in col_coordinate:
        dist = DistCalculation(sensor1_center,[row,col])
        if dist <= Rs:
            pts1_sensing_range_.append([row,col])

pts2_sensing_range_ = []
for row in row_coordinate:
    for col in col_coordinate:
        dist = DistCalculation(sensor2_center,[row,col])
        if dist <= Rs:
            pts2_sensing_range_.append([row,col])

training_X = []
training_Y = []

for pts in pts1_sensing_range_:
    training_X.append(pts)
    if 0 < pts[0] < 2 and 3 < pts[1] < 4:
        training_Y.append(1)
    elif 2 < pts[0] < 3 and 0 < pts[1] < 1:
        training_Y.append(1)
    elif 4 < pts[0] < 5 and 2 < pts[1] < 3:
        training_Y.append(1)
    else:
        training_Y.append(0)

for pts in pts2_sensing_range_:
    training_X.append(pts)
    if 0 < pts[0] < 2 and 3 < pts[1] < 4:
        training_Y.append(1)
    elif 2 < pts[0] < 3 and 0 < pts[1] < 1:
        training_Y.append(1)
    elif 4 < pts[0] < 5 and 2 < pts[1] < 3:
        training_Y.append(1)
    else:
        training_Y.append(0)

# for idx, pt in enumerate(training_X):
    # print("vertex ({},{}), the occupancy is {}".format(pt[0],pt[1],training_Y[idx]))

# Specify Gaussian Processes with fixed and optimized hyperparameters
gp_opt = GaussianProcessClassifier(kernel=1.0 * RBF(length_scale=1))
gp_opt.fit(training_X,training_Y)
print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))
print("Log Marginal Likelihood (optimized): %.3f"
      % gp_opt.log_marginal_likelihood(gp_opt.kernel_.theta))


# # Plot posteriors
# plt.figure(0)
# plt.scatter(X[:train_size][0], X[:train_size][1],y[:train_size], c='k', label="Train data",
#             edgecolors=(0, 0, 0))
# plt.scatter(X[train_size:][0], X[train_size:][1],y[train_size:], c='g', label="Test data",
#             edgecolors=(0, 0, 0))
# X1_ = np.linspace(1, 5, 5)
# X2_ = np.linspace(1, 5, 5)
# # print(X1_)
#
# X_ = np.asanyarray([[row,col] for row in X1_ for col in X2_])
#
#
#
# fig3 = plt.figure()
# ax = fig3.add_subplot(111,projection="3d")
#
# # print(gp_fix.predict_proba(X_[:]))
# ax.scatter(X_[:,0],X_[:,1], gp_fix.predict_proba(X_[:])[:,1], c='r', marker='o')
# ax.scatter(X_[:,0],X_[:,1], gp_opt.predict_proba(X_[:])[:,1], c='b', marker='o')
# ax.set_xlabel("X1")
# ax.set_ylabel("X2")
# ax.set_zlabel("Z")
#
#
#
# print(gp_fix.predict_proba(np.reshape(X_[0,:], (-1,2))))
# fig1 = plt.figure()
# ZZ = np.empty([5,5])
# for row in range(5):
#     for col in range(5):
#         K = [X1_[row],X2_[col]]
#         ZZ[row,col] = gp_fix.predict_proba(np.reshape(K,(-1,2)))[:,1]
#
#
# XX ,YY = np.mgrid[0:5:6j,0:5:6j]
#
# CMAP = plt.get_cmap('jet')
# plt.pcolormesh(XX,YY,ZZ,cmap=CMAP)
# cb = plt.colorbar(shrink = 1.0)
# # cb.ax.set_yticklabels(cb.ax.get_yticklabels(), fontsize=10)

XX = np.arange(0.5,5.5,1.0)
YY = np.arange(0.5,5.5,1.0)

print(YY)

fig = plt.figure()
ZZ = np.empty([5,5])
for idx1, row in enumerate(XX):
    for idx2,col in enumerate(YY):
        K = [row,col]
        p_occ = gp_opt.predict_proba(np.reshape(K,(-1,2)))[:,1]
        occ_ = gp_opt.predict(np.reshape(K,(-1,2)))
        ZZ[idx1,idx2] = p_occ
        plt.text(row,col,"{}".format(round(p_occ[0],3)), color='black',fontsize=10)
        if occ_ == 1:
            plt.text(row + 0.25, col+0.25, "OCC", color='black', fontsize=10)
        else:
            plt.text(row + 0.25, col + 0.25, "FREE", color='black', fontsize=10)
        # print("ZZ ({}, {}) represents the occupancy about ({}, {})".format(idx1,idx2,row,col))
        # print("P(occ) is {}".format(ZZ[idx1,idx2]))


# fig2 = plt.figure()
# ZZ = np.empty([26,26])
# for row in range(26):
#     for col in range(26):
#         K = [row_coordinate[row],col_coordinate[col]]
#         ZZ[row,col] = gp_opt.predict_proba(np.reshape(K,(-1,2)))[:,1]
#
XX ,YY = np.mgrid[0:5:6j,0:5:6j]

print(ZZ)

CMAP = plt.get_cmap('jet')
plt.pcolormesh(XX,YY,ZZ,cmap=CMAP)
cb = plt.colorbar(shrink = 1.0)



# Define the comparison
XX_c = np.arange(0.5,5.5,1.0)
YY_c = np.arange(0.5,5.5,1.0)
fig_c = plt.figure()
ZZ_c = np.empty([5,5])
ZZ_c = [[0.5,0.5,0.2,0.5,0.5],[0.5,0.2,0,0.8,0.5],[0.5,0.2,0.2,0.5,0.5],[0.2,0,0.2,0.5,0.5],[0.5,0.2,0.5,0.5,0.5]]
for idx1 in np.arange(5):
    for idx2 in np.arange(5):
        # print(XX[idx1],YY[idx2])
        plt.text(XX_c[idx1],YY_c[idx2],"{}".format(round(ZZ_c[idx1][idx2],3)), color='black',fontsize=10)

CMAP = plt.get_cmap('jet')
plt.pcolormesh(XX,YY,ZZ_c,cmap=CMAP)
cb = plt.colorbar(shrink = 1.0)
# cb.ax.set_yticklabels(cb.ax.get_yticklabels(), fontsize=30)

# plt.plot(X1_, X2_, gp_fix.predict_proba(X_[:][:]), 'r',
#          label="Initial kernel: %s" % gp_fix.kernel_)
# plt.plot(X1_, X2_, gp_opt.predict_proba(X_[:][:]), 'b',
#          label="Optimized kernel: %s" % gp_opt.kernel_)
# plt.xlabel("Feature")
# plt.ylabel("Class 1 probability")
# plt.xlim(0, 5)
# plt.ylim(-0.25, 1.5)
# plt.legend(loc="best")
#
# # # Plot LML landscape
# # plt.figure(1)
# # theta0 = np.logspace(0, 8, 30)
# # theta1 = np.logspace(-1, 1, 29)
# # Theta0, Theta1 = np.meshgrid(theta0, theta1)
# # LML = [[gp_opt.log_marginal_likelihood(np.log([Theta0[i, j], Theta1[i, j]]))
# #         for i in range(Theta0.shape[0])] for j in range(Theta0.shape[1])]
# # LML = np.array(LML).T
# # plt.plot(np.exp(gp_fix.kernel_.theta)[0], np.exp(gp_fix.kernel_.theta)[1],
# #          'ko', zorder=10)
# # plt.plot(np.exp(gp_opt.kernel_.theta)[0], np.exp(gp_opt.kernel_.theta)[1],
# #          'ko', zorder=10)
# # plt.pcolor(Theta0, Theta1, LML)
# # plt.xscale("log")
# # plt.yscale("log")
# # plt.colorbar()
# # plt.xlabel("Magnitude")
# # plt.ylabel("Length-scale")
# # plt.title("Log-marginal-likelihood")

plt.show()