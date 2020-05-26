
import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import LogNorm
import lcm

from graph_data import vertex_data
from graph_data import map_data
from graph_data import bayesian_opt_data

from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic,
                                              ExpSineSquared, DotProduct,
                                              ConstantKernel,WhiteKernel)
from bayesian_optimization_util import plot_approximation, plot_acquisition
from bayesian_lib import Vertex, Grid

def main():
    lc = lcm.LCM()
    graph = Grid(False)
    for i in range(1000):
        Graph_data = str(i)+"GraphData"
        subscription = lc.subscribe(Graph_data, graph.graph_data_handler)
    try:
        while True:
            lc.handle()
            graph.complexity_prob()
            graph.gpr()
            ig_prob_pair = graph.propose_location(graph.expected_improvement)
            # opt_x = [x[0] for x in opt_x]
            # print(opt_x)
            verts = graph.calculate_next_sensor_location(ig_prob_pair,4)
            # print("The reuslt of bayesian optimization is {}".format(opt_x))
            # print("ig prob pair is: ")
            # print(ig_prob_pair)
            print("The selected sensor locations are {}".format(verts))
            
            bayesian_flag_ = bayesian_opt_data()
            bayesian_flag_.bayesian_opt_flag_ = True
            bayesian_flag_.num_sensor_ = 4
            for pos in verts:
                bayesian_flag_.sensor_pos_.append(pos)
            # Publish the message
            lc.publish("BayesianOpt", bayesian_flag_.encode())
            print("Public the data from python to c+")
            print("================================================================")
    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)

if __name__ == '__main__':
    main()