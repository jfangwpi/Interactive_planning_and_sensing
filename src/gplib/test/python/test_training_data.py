import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import LogNorm

from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic,
                                              ExpSineSquared, DotProduct,
                                              ConstantKernel,WhiteKernel)

# training_X = [[0.0,0.2,0.5,0.5,0.5]]
# training_Y = [0.468446]  
# test_data = [[0.0,0.2,0.5,0.5,0.8]]

training_X = [[0.668741,0.668741,0.668741,0.8409,0.8409],[0.0,0.668741,0.668741,0.668741,0.8409]]
training_Y = [0.51785,0.47157]
# test_data = [[0.8409,0.8409,0.94574,0.94574,0.94574],[0.0,0.8409,0.94574,0.94574,0.94574]]
# test_data = [[0,0,0,0.8409,0.8409],[0,0,0,0.8409,0.94574]]
test_data = [[0.8409,0.8409,0.8409,0.8409,0.94574],[0.0,0.8409,0.8409,0.8409,0.8409]]


rbf = 1.0 * RBF(length_scale=1.0)  
matern = 1.0 * Matern(length_scale=1.0, length_scale_bounds=(1e-1, 10.0),
                nu=1.5)
gp_opt = GaussianProcessRegressor(kernel=rbf)
gp_opt.fit(training_X,training_Y)
print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))
print("Log Marginal Likelihood (optimized): %.3f"
    % gp_opt.log_marginal_likelihood(gp_opt.kernel_.theta))

y_mean, y_std = gp_opt.predict(test_data, return_std=True)
print("Mean value is {}".format(y_mean))
print("Standard diviation is {}".format(y_std))