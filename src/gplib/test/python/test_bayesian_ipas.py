import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import LogNorm
import lcm

from graph_data import vertex_data
from graph_data import map_data

from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic,
                                              ExpSineSquared, DotProduct,
                                              ConstantKernel,WhiteKernel)
from bayesian_optimization_util import plot_approximation, plot_acquisition
from bayesian_lib import Vertex, Grid



def main():
    grid_cols = 20
    grid_rows = 20

    lc = lcm.LCM()
    true_graph = Grid(True)
    for i in range(num_cases+1):
        Graph_data = str(i)+"GraphData"
        subscription = lc.subscribe(Graph_data, true_graph.graph_data_handler)
    msg_flag = prediction_data()
    bayes_data_ = []
    ave_opt = 0
    ave_relative_err = 0
    ave_iteration = 0
    ave_samples = 0
    correct_case = 0
    try:
        while True:
            lc.handle()
            data = []
            pred_graph = Grid(False,ROIs,nz_ig)
            pred_graph.duplicate_true_graph(true_graph)
            pred_graph.adjacent_prob(true_graph)
            pred_graph.training_samples(samples_idx,true_graph)
            pred_graph.complexity_prob()
            bayes_iter = 0
            while bayes_iter < BayesianMaxIter:
                bayes_iter = bayes_iter + 1
                gpr = pred_graph.gp(true_graph, samples_idx)
                true_graph.calculate_ave_L2_norm()
                true_graph.maximum_IG()
                pred_graph.maximum_IG()
                if true_graph.max_ig_vertex_ == pred_graph.max_ig_vertex_ or abs(pred_graph.vertex_[true_graph.max_ig_vertex_].ig_-true_graph.max_ig_)<=5e-3:
                    print("============== Find the same max ig ===============")
                    break
                next_sample_x = pred_graph.propose_location(pred_graph.expected_improvement)
                next_sample_x = [round(i[0],2) for i in next_sample_x]
                print("======================================================================")
                print("The next X sample selected by Bayesian Opt is {}".format(next_sample_x))
                next_sample_y = pred_graph.locate_vertex(next_sample_x,true_graph)
                print("The corresponding ig for the next sample is {}".format(next_sample_y))

                pred_graph.training_x_.append(next_sample_x)
                pred_graph.training_y_.append(next_sample_y)
            true_graph.maximum_IG()
            pred_graph.maximum_IG()
            pred_graph.bayesian_iter_ = bayes_iter
            pred_graph.bayesian_samples_ = bayes_iter+len(samples_idx)
            pred_graph.bayesian_opt_ = round(pred_graph.max_ig_,3)/round(true_graph.max_ig_,3)
            pred_graph.bayesian_relative_err_ = abs(true_graph.max_ig_-pred_graph.max_ig_)/true_graph.max_ig_
            print("===========================================================================")
            print("===========================================================================")
            print("===========================================================================")
            print("Bayesian Opt terminates at T = {}".format(pred_graph.bayesian_iter_))
            print("Required samples number is {}".format(pred_graph.bayesian_samples_))
            print("The maximum IG for true graph is {}, the maximum IG for predicted graph is {}".format(true_graph.max_ig_,pred_graph.max_ig_))
            print("The maximum IG vertex for true graph is {}".format(true_graph.max_ig_vertex_))
            print("The maximum IG vertex for predicted graph is {}".format(pred_graph.max_ig_vertex_))
            print("The optimality for the Bayesian optimization is {}".format(pred_graph.bayesian_opt_))
            print("============================================================================")
            print("============================================================================")
            print("============================================================================")
            msg_flag.bayesian_opt_flag = True
            lc.publish("BayesianChannel", msg_flag.encode())
            print("Publish the msg")
            data.append(case_idx)
            data.append(true_graph.max_ig_)
            data.append(pred_graph.max_ig_)
            data.append(pred_graph.bayesian_opt_)
            ave_opt = ave_opt+pred_graph.bayesian_opt_
            ave_relative_err = ave_relative_err + pred_graph.bayesian_relative_err_
            ave_iteration = ave_iteration + pred_graph.bayesian_iter_
            ave_samples = ave_samples + pred_graph.bayesian_samples_
            data.append(pred_graph.bayesian_iter_)
            data.append(pred_graph.bayesian_samples_)
            data.append(true_graph.max_ig_vertex_)
            data.append(pred_graph.max_ig_vertex_)
            data.append(pred_graph.bayesian_relative_err_)
            bayes_data_.append(data)

            # true_vt = true_graph.max_ig_vertex_
            # if true_vt == pred_graph.max_ig_vertex_ or abs(pred_graph.vertex_[true_vt].ig_-true_graph.max_ig_)<=10e-2:
            #     correct_case = correct_case+1
            
            case_idx = case_idx+1
            if case_idx >= num_cases:
                break


        print("The average optimality is {}".format(ave_opt/num_cases))
        print("The average relative error is {}".format(ave_relative_err/num_cases))
        print("The average iteration is {}".format(ave_iteration/num_cases))
        print("The average samples is {}".format(ave_samples/num_cases))
        print("Correct cases are {}".format(correct_case))
        print("Start to write into cvs")
        with open('bayes_opt_data.cvs',mode='w') as csv_file:
            fieldnames = ['#Case','TrueMAXIG','PredictedMAXIG',
                'Optimality','Iteration','#samples',
                'TrueMAXVert','PredictMAXVert','RelativeError']
            writer = csv.DictWriter(csv_file,fieldnames=fieldnames)
            writer.writeheader()
            for case in bayes_data_:
                writer.writerow({'#Case':case[0],'TrueMAXIG':case[1],'PredictedMAXIG':case[2],
                'Optimality':case[3],'Iteration':case[4],'#samples':case[5],
                'TrueMAXVert':case[6],'PredictMAXVert':case[7],'RelativeError':case[8]})

    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)
    


if __name__ == '__main__':
    main()