import numpy as np
import math
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process.kernels import RBF
from sklearn.gaussian_process.kernels import Matern


# This file is used to test can GP represent certain occupancy grid map with given sensors position and measurements
num_row = 30
num_col = 30

x = np.arange(0.5,num_row + 0.5,1.0)
y = np.arange(0.5,num_col + 0.5,1.0)

# Define obstacle
obs1_x = np.arange(18.5,27.5,1.0)
obs1_y = np.arange(24.5,30.5,1.0)
obs1 = [[idx1,idx2] for idx1 in obs1_x for idx2 in obs1_y]

obs2_x = np.arange(0.5,8.5,1.0)
obs2_y = np.arange(13.5,24.5,1.0)
obs2 = [[idx1,idx2] for idx1 in obs2_x for idx2 in obs2_y]

obs3_x = np.arange(15.5,30.5,1.0)
obs3_y = np.arange(17.5,21.5,1.0)
obs3 = [[idx1,idx2] for idx1 in obs3_x for idx2 in obs3_y]

obs4_x = np.arange(27.5,30.5,1.0)
obs4_y = np.arange(12.5,17.5,1.0)
obs4 = [[idx1,idx2] for idx1 in obs4_x for idx2 in obs4_y]

obs5_x = np.arange(0.5,5.5,1.0)
obs5_y = np.arange(5.5,10.5,1.0)
obs5 = [[idx1,idx2] for idx1 in obs5_x for idx2 in obs5_y]

obs6_x = np.arange(9.5,20.5,1.0)
obs6_y = np.arange(0.5,6.5,1.0)
obs6 = [[idx1,idx2] for idx1 in obs6_x for idx2 in obs6_y]

obs7_x = np.arange(23.5,28.5,1.0)
obs7_y = np.arange(3.5,7.5,1.0)
obs7 = [[idx1,idx2] for idx1 in obs7_x for idx2 in obs7_y]

obs_ = obs1 + obs2 + obs3 + obs4 + obs5 + obs6 + obs7
print("the length of obs is {}".format(len(obs_)))
training_X = []
training_Y = []

# Sensors position
sensors_1 = [396,337,309,865,275]
sensors_2 = [691,632,92,667,664]
sensors_3 = [843,813,443,595,188]
sensors_4 = [545,514,137,513,107]
sensors_5 = [725,755,756,786,745]
sensors_6 = [804,48,774,127,128]
sensors_7 = [686,51,806,655,716]
sensors_8 = [54,688,718,282,197]
sensors_9 = [411,808,809,779,729]
sensors_10 = [605,31,541,481,670]
sensors_11 = [841,369,505,255,46]
sensors_12 = [249,278,248,94,219]
sensors_13 = [472,638,168,636,228]
sensors_14 = [868,156,845,784,814]
sensors_15 = [440,253,313,252,566]
sensors_16 = [90,60,748,63,624]
sensors_17 = [338,444.310,872,512]
sensors_18 = [257,226,159,413,446]
sensors_19 = [839,696,700,77,224]
sensors_20 = [399,669,32,281,1]
sensors_21 = [840,506,535,536,475]

sensors = sensors_1 + sensors_2 + sensors_3 + sensors_4 + sensors_5 + sensors_6 + sensors_7 + sensors_8 + sensors_9 + sensors_10
sensors = sensors + sensors_11 + sensors_12 + sensors_13 + sensors_14 + sensors_15 + sensors_16 + sensors_17 + sensors_18 + sensors_19 + sensors_20 + sensors_21
sensors_X = []
for ss in sensors:
    col = ss%num_row + 0.5
    row = (30 - (ss - col + 0.5)/num_row) - 0.5
    sensors_X.append([row,col])

print(sensors_X)
# sensors_X = [[0.5,6.5],[0.5,1.5],[1.5,8.5],[1.5,7.5],[1.5,6.5],[1.5,5.5],[1.5,3.5],[1.5,1.5],[2.5,9.5],[2.5,6.5],[2.5,5.5],[2.5,4.5],[3.5,8.5],[3.5,5.5],[3.5,4.5],[3.5,3.5],[3.5,1.5],[4.5,6.5],[4.5,4.5],[4.5,1.5],[5.5,7.5],[5.5,4.5],[5.5,2.5],[6.5,5.5],[6.5,3.5],[7.5,7.5],[7.5,5.5],[8.5,8.5],[8.5,6.5]]
print("The # of sensors is {}".format(len(sensors_X)))
for c_sensor in sensors_X:
    s_x = c_sensor[0]
    s_y = c_sensor[1]
    if [s_x,s_y] not in training_X:
        training_X.append([s_x,s_y])
        if c_sensor not in obs_:
            training_Y.append(0)
        else:
            training_Y.append(1)

    # Up
    n_y = s_y + 1
    if n_y < num_row + 0.5:
        if [s_x,n_y] not in training_X:
            training_X.append([s_x,n_y])
            if [s_x,n_y] not in obs_:
                training_Y.append(0)
            else:
                training_Y.append(1)

    # Down
    n_y = s_y - 1
    if n_y >= 0.5:
        if [s_x, n_y] not in training_X:
            training_X.append([s_x,n_y])
            if [s_x,n_y] not in obs_:
                training_Y.append(0)
            else:
                training_Y.append(1)
    
    # Left
    n_x = s_x - 1
    if n_x >= 0.5:
        if [n_x,s_y] not in training_X:
            training_X.append([n_x,s_y])
            if [n_x,s_y] not in obs_:
                training_Y.append(0)
            else:
                training_Y.append(1)

    # Right
    n_x = s_x + 1 
    if n_x < num_col + 0.5:
        if [n_x,s_y] not in training_X:
            training_X.append([n_x,s_y])
            if [n_x,s_y] not in obs_:
                training_Y.append(0)
            else:
                training_Y.append(1)

# training_X.append([0.5,9.5])
# training_Y.append(0)

# training_X.append([0.5,4.5])
# training_Y.append(0)

# training_X.append([5.5,0.5])
# training_Y.append(0)

# training_X.append([7.5,9.5])
# training_Y.append(0)

print(len(training_X))
print(len(training_Y))

# Specify Gaussian Processes with fixed and optimized hyperparameters
gp_opt = GaussianProcessClassifier(RBF(length_scale=1.0))
gp_opt.fit(training_X,training_Y)
print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))
# print("Log Marginal Likelihood (optimized): %.3f"
#       % gp_opt.log_marginal_likelihood(gp_opt.kernel_.theta))


# print("The probability of occupancy is {}".format(p_occ))

fig = plt.figure()
ZZ = np.empty([30,30])
for idx1, row in enumerate(x):
    for idx2,col in enumerate(y):
        K = [row,col]
        if K in training_X:
            ZZ[idx1,idx2] = 0.0
        else:
        # print("({},{})".format(row,col))
        # print("({},{})".format(idx1,idx2))
            p_occ = gp_opt.predict_proba(np.reshape(K,(-1,2)))[:,1]
            
            occ_ = gp_opt.predict(np.reshape(K,(-1,2)))
            ZZ[idx1,idx2] = p_occ
            plt.text(row,col,"{}".format(round(p_occ[0],3)), color='black',fontsize=5)
            if occ_ == 1:
                plt.text(row + 0.25, col+0.25, "OCC", color='black', fontsize=5)
            else:
                plt.text(row + 0.25, col + 0.25, "FREE", color='black', fontsize=5)

XX ,YY = np.mgrid[0:30:31j,0:30:31j]

CMAP = plt.get_cmap('jet')
plt.pcolormesh(XX,YY,ZZ,cmap=CMAP)
cb = plt.colorbar(shrink = 1.0)

plt.show()

