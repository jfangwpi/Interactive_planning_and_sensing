import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from scipy.optimize import minimize
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic,
                                              ExpSineSquared, DotProduct,
                                              ConstantKernel,WhiteKernel)
from bayesian_optimization_util import plot_approximation, plot_acquisition    
import random
from graph_data import vertex_data
from graph_data import map_data
from graph_data import prediction_data
import lcm
from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
import csv
import math

grid_rows = 30
grid_cols = 30
samples_idx = []
dir = [[-1,0],[1,0],[0,-1],[0,1]]
INT_MIN = -10
INT_MAX = 10

MAX_SAMPLES = 100
MAX_ITER = 10e3
BayesianMaxIter = grid_rows*grid_cols
TOL_AVE_L2 = 10e-4

class Vertex(object):
    def __init__(self,id,pos_x,pos_y,p,ig):
        self.idx_ = id
        self.coordinate_ = [pos_x,pos_y] # number of row, col
        self.p_ = p
        self.surrounding_p_ = 0
        self.occupancy_ = "UNKNOWN"

        self.ig_ = ig
        # Require by the estimate ig map
        self.ig_ub_ = ig
        self.ig_lb_ = ig
        self.ig_pred_ = False
        self.L2_error_ = 0.0
        # Check the simulation
        self.isROIs_ = False
        self.isSamples_ = False

class Grid(object):
    def __init__(self,accurate_graph=False,ROIS=[],NZIG=[]):
        self.rows_ = grid_rows
        self.cols_ = grid_cols
        self.vertex_ = {}
        self.graph_init()
        self.graph_status_ = accurate_graph
        self.L2_error_total_ = 0.0

        self.ROIs_ = ROIS
        self.NZIG_ = NZIG
        self.nzig_prob_ = []

        self.max_ig_ = 0.0
        self.max_ig_vertex_ = []

        if not accurate_graph:
            self.gpr_ = None
            self.samples_ = []
            self.training_x_ = []
            self.training_y_ = []
            self.poss_probs_ = []
            self.bayesian_iter_ = 0
            self.bayesian_opt_ = 0.0
            self.bayesian_samples_ = 0
            self.bayesian_relative_err_ = 0.0
        
    def graph_init(self):
        for idx in range(self.rows_*self.cols_):
            coord_x = self.rows_ - int(idx/self.rows_) - 1
            coord_y = idx%self.rows_
            vert = Vertex(idx,coord_x,coord_y,0.5,0.0) # Initialize the vertex with p=0.5, ig=0.0
            # print("Vertex {}, coordinate is ({}, {})".format(idx, coord_x,coord_y))
            self.vertex_[idx] = vert

    def set_vertex(self, id, occ):
        vert = self.vertex_[id]
        if occ == "FREE":
            vert.p_ = 0.0
            vert.occupancy_ = "FREE"
        elif occ == "OCCUPIED":
            vert.p_ = 1.0
            vert.occupancy_ = "OCCUPIED"
    
    def set_probability(self,id,p):
        vert = self.vertex_[id]
        if p == 0.0:
            self.set_vertex(id,"FREE")
        elif p == 1.0:
            self.set_vertex(id,"OCCUPIED")
        else:
            vert.p_ = p

    def graph_data_handler(self, channel, data):
        self.ROIs_ = []
        self.NZIG_ = []
        self.samples_ = []
        msg = map_data.decode(data)
        for cell in msg.vertex_:
            idx = cell.idx_
            if cell.isROIs_:
                self.ROIs_.append(idx)
                self.vertex_[idx].isROIs_ = True
            if cell.isNZIG_:
                self.NZIG_.append(idx)
            if cell.isSamples_: 
                self.samples_.append(idx)
                self.vertex_[idx].ig_ = cell.ig_   
                self.vertex_[idx].isSamples_ = True      
            # else:
            #     self.vertex_[idx].ig_ = math.inf
            self.set_probability(idx,round(cell.p_,3))
       

        self.L2_error_total_ = 0.0
        self.max_ig_ = 0.0
        self.max_ig_vertex_ = []
        self.bayesian_opt_ = 0.0
        self.bayesian_relative_err_ = 0.0

        # print("The size of nzig is {}".format(len(self.NZIG_)))
        for idx in self.NZIG_:
            self.adjacent_prob(idx)
            # print("Vertex {}. Surronding prob is {}".format(idx,self.vertex_[idx].surrounding_p_))
            # print("Vertex {}, Prob is {}".format(idx,self.vertex_[idx].p_))

        # Update the the surronding probability for each cell inside of ROIs and construct training_x
        for idx in self.samples_:
            # self.adjacent_prob(idx)
            # print("Vertex {}, surrounding probability is {}".format(idx,self.vertex_[idx].surrounding_p_))
            if self.vertex_[idx].surrounding_p_ not in self.training_x_:
                self.training_x_.append(self.vertex_[idx].surrounding_p_)
                self.training_y_.append([self.vertex_[idx].ig_])
                # self.training_x_.sort()
                # self.training_y_.sort()
                # self.training_y_.reverse()
                for kk,item in enumerate(self.training_x_):
                    if item == self.vertex_[idx].surrounding_p_:
                        if self.training_y_[kk] != [self.vertex_[idx].ig_]:
                            self.training_x_.remove(self.vertex_[idx].surrounding_p_)
                            self.training_y_.remove([self.vertex_[idx].ig_])

        
        # for idx,x in enumerate(self.training_x_):
        #     print("sample X: {}, sample Y: {}".format(x,self.training_y_[idx]))
        # plt.plot(self.training_x_,self.training_y_,'b*') 
        # plt.show()   
        
            
                

        # print("Collected all data from LCM for bayesian optimization.")
        # for idx in self.vertex_.keys():
        #     vert = self.vertex_[idx]
        #     if vert.isROIs_:
        #         print("Vertex: {}, Inside of ROIs".format(vert.idx_))
        #     else:
        #         print("Vertex: {}, Outside of ROIs".format(vert.idx_))

        #     if vert.isSamples_:
        #         print("Vertex: {}, Is sample data".format(vert.idx_))
        #     else:
        #         print("Vertex: {}, Not sample data".format(vert.idx_))
            
        #     print("============================================")

        print("The training data is {}".format(len(self.training_x_)))
        # for idx, item in enumerate(self.training_x_):
        #     print("Prob: {}, IG: {}".format(item,self.training_y_[idx]))
        


    # Update the surrounding probabilities for each vertex
    def adjacent_prob(self,vtIdx):
        vert = self.vertex_[vtIdx]        
        x = vert.coordinate_[1]
        y = vert.coordinate_[0]
        # if  vtIdx in self.ROIs_:
        #     neigh_p = [vert.p_]
        # else:
        #     neigh_p = [INT_MAX]
        neigh_p = []
        # print("Vertex {}'s neighbors are: ".format(vtIdx))
        for d in dir:
            new_x = x+d[0]
            new_y = y+d[1]
            nb_idx = (self.rows_-new_y-1)*self.cols_+new_x
            # print(nb_idx)
            if 0 <= new_x and new_x < self.cols_ and 0<= new_y and new_y < self.rows_ and nb_idx in self.ROIs_: 
                # neigh_p.append(round(0.5-abs(self.vertex_[nb_idx].p_-0.5),3))
                p = self.vertex_[nb_idx].p_
                q = 1.0-p
                if p == 1.0 or p == 0.0:
                    p_ig = 0
                else:
                    p_ig = -p*np.log(p)-q*np.log(q)
                neigh_p.append(round(p_ig,3))
            elif 0 <= new_x and new_x < self.cols_ and 0<= new_y and new_y < self.rows_:
                # neigh_p.append(INT_MAX)
                neigh_p.append(0)
            else:
                # neigh_p.append(INT_MIN)
                neigh_p.append(0)

        sum_neighboring_ig = [sum(neigh_p)]

        if  vtIdx in self.ROIs_:
            # neigh_p = [round(0.5-abs(vert.p_-0.5),3)]
            p = vert.p_
            q = 1.0-p
            if p == 1.0 or p == 0.0:
                p_ig = 0
            else:
                p_ig = -p*np.log(p)-q*np.log(q)
            sum_neighboring_ig.append(round(p_ig,3))
        else:
            sum_neighboring_ig.append(0)
           
        neigh_p = [-x for x in neigh_p]
        # self.vertex_[vtIdx].surrounding_p_ = neigh_p
        self.vertex_[vtIdx].surrounding_p_ = sum_neighboring_ig
        

    # Update the training datas
    def training_samples(self,samples_idx,true_graph):
        for idx in self.samples_:
            if self.vertex_[idx].surrounding_p_ not in self.training_x_:
                # print("The index is {}".format(idx))
                self.training_x_.append(self.vertex_[idx].surrounding_p_)
                # print("The training sample is {}".format(self.training_x_[-1]))
                self.training_y_.append(true_graph.vertex_[idx].ig_)
                # print("The training y is {}".format(self.training_y_[-1]))

    def gpr(self):
        rbf = 1.0 * RBF(length_scale=1.0)  
        matern = 1.0 * Matern(length_scale=1.0, length_scale_bounds=(1e-8, 5.0),
                        nu=1.5)
        gp_opt = GaussianProcessRegressor(kernel=matern)
        gp_opt.fit(self.training_x_,self.training_y_)
        self.gpr_ = gp_opt
        print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))
        print("Log Marginal Likelihood (optimized): %.3f"
            %gp_opt.log_marginal_likelihood(gp_opt.kernel_.theta))

        # x1_ = [i for i in range(self.rows_+1)]
        # x2_ = [j for j in range(self.cols_+1)]
        # X1_, X2_ = np.meshgrid(x1_,x2_)

        # y_mean = np.empty([self.rows_+1,self.cols_+1])
        # y_true = np.empty([self.rows_+1,self.cols_+1])
        # y_std = np.empty([self.rows_+1,self.cols_+1])
        # y_mean_u = np.empty([self.rows_+1,self.cols_+1])
        # y_mean_d = np.empty([self.rows_+1,self.cols_+1])
        # for i in range(self.rows_+1):
        #     for j in range(self.cols_+1):
        #         cur_idx = (self.rows_-X1_[i][j]-1)*self.cols_+X2_[i][j]
        #         # print("X: {}, Y: {}, idx: {}".format(X2_[i][j], X1_[i][j],cur_idx))
        #         if X1_[i][j]<self.rows_ and X1_[i][j]>=0 and X2_[i][j]<self.cols_ and X2_[i][j]>=0 and cur_idx in nz_ig:
        #             y_mean[X2_[i][j],X1_[i][j]], y_std[X2_[i][j],X1_[i][j]] = gp_opt.predict([self.vertex_[cur_idx].surrounding_p_], return_std=True)
        #             # print("Prediction ========================")
        #             # print("Vertex {}, X:{}, Y:{}".format(cur_idx,X2_[i][j],X1_[i][j]))
        #             # print("Testing data is {}".format(self.vertex_[cur_idx].probs_))
        #             # print("Predicted IG {}".format(y_mean[X2_[i][j],X1_[i][j]]))
        #         else:
        #             y_mean[X2_[i][j] ,X1_[i][j]] = 0
        #             y_std[X2_[i][j],X1_[i][j]] = 0.0
                
        #         y_mean_u[X2_[i][j],X1_[i][j]] = y_mean[X2_[i][j],X1_[i][j]] + y_std[X2_[i][j],X1_[i][j]]
        #         y_mean_d[X2_[i][j],X1_[i][j]] = y_mean[X2_[i][j],X1_[i][j]] - y_std[X2_[i][j],X1_[i][j]]

                
        #         if X2_[i][j]<self.cols_ and X1_[i][j]<self.rows_ and X1_[i][j]>=0 and X2_[i][j]>=0:
        #             idx_ = (self.rows_-X1_[i][j]-1)*self.cols_+X2_[i][j]
        #             y_true[X2_[i][j],X1_[i][j]] = true_graph.vertex_[idx_].ig_
        #             self.vertex_[idx_].ig_ = y_mean[X2_[i][j],X1_[i][j]]
        #             true_graph.vertex_[idx_].L2_error_ = (true_graph.vertex_[idx_].ig_ - self.vertex_[idx_].ig_)**2
               
        #             self.vertex_[idx_].ig_ub_ = y_mean[X2_[i][j],X1_[i][j]] + y_std[X2_[i][j],X1_[i][j]]
        #             self.vertex_[idx_].ig_lb_ = y_mean[X2_[i][j],X1_[i][j]] - y_std[X2_[i][j],X1_[i][j]]
        #             if round(true_graph.vertex_[idx_].ig_,3) <= round(self.vertex_[idx_].ig_ub_,3) and round(true_graph.vertex_[idx_].ig_,3) >= round(self.vertex_[idx_].ig_lb_,3):
        #                 self.vertex_[idx_].ig_pred_ = True
        #             # print("Idx {}, Coordinate {}, {}, IG is {}".format(idx_,X2_[i][j],X1_[i][j],self.vertex_[idx_].ig_)
    
    def calculate_L2_error(self):
        self.L2_error_total_ = 0.0
        for vt in self.vertex_.keys():
            self.L2_error_total_ += self.vertex_[vt].L2_error_
        self.L2_error_total_ = np.square(self.L2_error_total_)
        print("The L2 norm error is {}".format(self.L2_error_total_))

    def calculate_ave_L2_norm(self):
        self.calculate_L2_error()
        self.ave_L2_norm_ = self.L2_error_total_/(len(self.nz_ig_zones_))

    def prob_valid(self,sample_p):
        for v_idx in self.vertex_.keys():
            if v_idx in self.ROIs_ and self.vertex_[v_idx].surrounding_p_ == sample_p:
                return True
        return False

    def calculate_next_sensor_location(self,opt_prbs,nsensors):
        sensor_pos_ = []
        for ig in sorted(opt_prbs.keys(),reverse=True):
            print("The ig is {}".format(ig))
            for idx in self.NZIG_:
                # print(idx)
                for p in opt_prbs[ig]: 
                    if len(sensor_pos_) < nsensors:
                        if self.vertex_[idx].surrounding_p_ == p and idx not in sensor_pos_:
                            # print("Surronding prob is {}, the idx is {}".format(self.vertex_[idx].surrounding_p_,idx))
                            sensor_pos_.append(idx)
        return sensor_pos_

    def complexity_prob(self):
        self.poss_probs_ = []
        for idx in self.ROIs_:
            if self.vertex_[idx].p_ not in self.poss_probs_:
                self.poss_probs_.append(self.vertex_[idx].p_)
        # self.poss_probs_.append(1)
        # self.poss_probs_.append(-1)
        # print("The possible complexity of probability is {}".format(self.poss_probs_))

    # Bayesian Optimization
    def expected_improvement(self,X,xi=0.01):
        mu, sigma = self.gpr_.predict(X, return_std=True)
        mu_sample = self.gpr_.predict(self.training_x_)

        sigma = sigma.reshape(-1, 1)
        
        # Needed for noise-based model,
        # otherwise use np.max(Y_sample).
        # See also section 2.4 in [...]
        mu_sample_opt = np.max(mu_sample)
        # print("testing X is ")
        # print(X)
        # print("Mean for X")
        # print(mu)

        with np.errstate(divide='warn'):
            imp = mu - mu_sample_opt - xi
            if sigma == 0:
                ei = 0
            else:
                Z = imp / sigma
                ei = imp * norm.cdf(Z) + sigma * norm.pdf(Z)
                # print("Expect improvement is")
                # print(ei)
                ei[sigma == 0.0] = 0.0
        # if X not in self.training_x_:
        # print("For test data {}, mu is {}, sigma is{}, ei is {}".format(X,mu,sigma,ei))
        return mu+ei,sigma,ei

    def propose_location(self, acquisition):
        '''
        Proposes the next sampling point by optimizing the acquisition function.
        
        Args:
            acquisition: Acquisition function.
            X_sample: Sample locations (n x d).
            Y_sample: Sample values (n x 1).
            gpr: A GaussianProcessRegressor fitted to samples.

        Returns:
            Location of the acquisition function maximum.
        '''
        # dim = len(self.training_x_[0])
        # min_val = -np.inf
        # min_x = None
        
        # def min_obj(X):
        #     # Minimization objective is the negative acquisition function
        #     return acquisition(X.reshape(-1, dim))

        # def                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          (x):
        #     eqn = 1
        #     for p in self.poss_probs_:
        #         eqn *= (x-p)
        #     return eqn
            # return (x-0.0)*(x-0.5)*(x-0.8)*(x-0.2)*(x-INT_MAX)*(x-INT_MIN)*(x-0.3)*(x-0.4)
        
        # Find the best optimum by starting from n_restart different random points.
        # random_samples = []

        # iteration = 0
        # while len(self.rdSamples_) < MAX_SAMPLES and iteration < MAX_ITER:
        #     sample = []
        #     iteration = iteration+1
        #     while len(sample) < dim:
        #         idx = random.randint(0,len(self.poss_probs_)-1)
        #         sample.append(self.poss_probs_[idx])
        #     sample.sort()
        #     flag_ = self.prob_valid(sample)
        #     if flag_ and sample not in self.rdSamples_:
        #         self.rdSamples_.append(sample)
        
        
        # print("The random generated samples are {}".format(self.rdSamples_))
        ig_pair = {}
        self.nzig_surronding_prob()
        for x0 in self.nzig_prob_:
            # cons = ({'type':'eq','fun':eq_constraint})
            # print("====================================")
            # print("initial state is {}".format(x0))
            # res = minimize(min_obj, x0=x0, constraints=cons) 
            # res = minimize(min_obj, x0=x0)
            key, sigma, ei= self.expected_improvement([x0])
            key = key[0][0]
            val = x0
            
            if key not in ig_pair.keys():
                ig_pair[key] = []
                ig_pair[key].append(val)
            else:
                if val not in ig_pair[key]:
                    ig_pair[key].append(val)
            # val = [round(x,3) for x in val]
            # if val not in self.training_x_:
            # print("The ig value is {}, sigma is {}, ei is {} and the prob is {}".format(key,sigma,ei,val))
            # print("The ig value is {},and the prob is {}".format(key,val))

        #     if res.fun > min_val:
        #         min_val = res.fun
        #         min_x = res.x           
        # for item in ig_pair.keys():
        #     print("The ig is {}, the corresponding probs is {}".format(item, ig_pair[item]))
        # print("maximum IG from Bayesian OPT is {}".format(min_val))
        # return min_x.reshape(-1, 1)
        return ig_pair

    def nzig_surronding_prob(self):
        self.nzig_prob_ = []
        for x in self.NZIG_:
            if self.vertex_[x].surrounding_p_ not in self.nzig_prob_:
                self.nzig_prob_.append(self.vertex_[x].surrounding_p_)


    def duplicate_true_graph(self,true_graph):
        for idx in self.vertex_.keys():
            self.vertex_[idx].p_ = true_graph.vertex_[idx].p_
            self.vertex_[idx].ig_ = true_graph.vertex_[idx].ig_
            self.vertex_[idx].occupancy_ = true_graph.vertex_[idx].occupancy_

    def locate_true_ig(self,p,true_graph):
        for idx in self.vertex_.keys():
            if self.vertex_[idx].surrounding_p_ == p:
                # print("The located vertex is {}".format(idx))
                return true_graph.vertex_[idx].ig_
        return 0.0

    def maximum_IG(self):
        max_ig = -np.inf
        max_vt = 0
        for idx in self.vertex_.keys():
            if self.vertex_[idx].ig_ > max_ig:
                max_ig = self.vertex_[idx].ig_
                max_vt = idx
        self.max_ig_ = round(max_ig,3)
        self.max_ig_vertex_ = max_vt

    def graph_vis(self,type="Origin"):
        # plt.figure()
        if self.graph_status_:
            graph_title_ = "Accurate Graph -- " + type
        else:
            graph_title_ = "Unknown Graph -- " + type 
        plt.title(graph_title_)
        plt.grid(color='black',linestyle='-',linewidth=1)
        plt.xlim(0,self.cols_)
        plt.ylim(0,self.rows_)
        plt.xticks(np.arange(0,self.cols_+1,1))
        plt.yticks(np.arange(0,self.rows_+1,1))
        for idx in range(self.rows_*self.cols_):
            vert = self.vertex_[idx]
            if type == "Mean":
                plt.text(vert.coordinate_[1] + 0.5, vert.coordinate_[0] + 0.5,"{}".format(round(vert.ig_,5)), color='black', fontsize=12, weight='bold')
            elif type == "MeanUpBound":
                plt.text(vert.coordinate_[1] + 0.5, vert.coordinate_[0] + 0.5,"{}".format(round(vert.ig_ub_,3)), color='black', fontsize=12,weight='bold')
            elif type == "MeanLowBound":
                plt.text(vert.coordinate_[1] + 0.5, vert.coordinate_[0] + 0.5,"{}".format(round(vert.ig_lb_,3)), color='black', fontsize=12,weight='bold')
            elif type == "InRange":
                plt.text(vert.coordinate_[1] + 0.5, vert.coordinate_[0] + 0.5,"{}".format(round(vert.ig_pred_,3)), color='black', fontsize=12,weight='bold')
            elif type == "Origin":
                plt.text(vert.coordinate_[1] + 0.5, vert.coordinate_[0] + 0.5,"{}".format(round(vert.p_,3)), color='black', fontsize=12,weight='bold')
      
    def ig_prediction_vis(self):
        self.graph_vis()
        self.graph_vis("MeanUpBound")
        self.graph_vis("MeanLowBound")
        self.graph_vis("InRange")

        # plt.figure()
        plt.subplots_adjust(left=0.03,right=1.0,wspace=0.02)
        plt.subplot(1,2,1)
        plt.title("IG prediction -- mean")
        CMAP = plt.get_cmap('coolwarm')
        plt.pcolormesh(X2_,X1_,y_mean,cmap=CMAP)
        cb = plt.colorbar(shrink = 1.0)
        plt.subplot(1,2,2)
        plt.title("True IG")
        CMAP = plt.get_cmap('coolwarm')
        plt.pcolormesh(X2_,X1_,y_true,cmap=CMAP)
        cb = plt.colorbar(shrink = 1.0)  

        # plt.figure()
        plt.title("Diviation")
        plt.pcolormesh(X2_,X1_,y_std,cmap=CMAP)
        cb = plt.colorbar(shrink = 1.0)
        # plt.show()
        
        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.plot_surface(X1_,X2_,y_mean,cmap=cm.coolwarm)
        # ax.plot_surface(X1_,X2_,y_mean_u,color='r')
        # ax.plot_surface(X1_,X2_,y_mean_d,color='b')
        # for id in true_graph.vertex_.keys():
        #     ax.scatter(true_graph.vertex_[id].coordinate_[1],true_graph.vertex_[id].coordinate_[0],true_graph.vertex_[id].ig_,'bo')