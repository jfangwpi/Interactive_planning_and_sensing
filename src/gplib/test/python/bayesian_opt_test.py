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

grid_rows = 40  
grid_cols = 40
samples_idx = [4,10]
dir = [[-1,0],[1,0],[0,-1],[0,1]]
INT_MIN = -10
INT_MAX = -INT_MIN
# Define the ROIs
# 10*10 map
nz_ig = []
ROIs = []
for r in range(0,grid_rows*grid_cols,1):
    nz_ig.append(r)
ROIs = nz_ig

MAX_SAMPLES = 25
MAX_ITER = 10e4
BayesianMaxIter = grid_rows*grid_cols
TOL_AVE_L2 = 10e-4
NUM_NZ_IG = len(nz_ig)

class Vertex(object):
    def __init__(self,id,pos_x,pos_y,p,ig):
        self.idx_ = id
        self.coordinate_ = [pos_x,pos_y] # number of row, col
        self.p_ = p
        self.probs_ = []
        self.occupancy_ = "UNKNOWN"

        self.ig_ = ig
        self.ig_ub_ = ig
        self.ig_lb_ = ig
        self.ig_pred_ = False

        self.L2_error_ = 0.0
        
class Grid(object):
    def __init__(self,accurate_graph=False,ROIS=[],NZIG=[]):
        self.rows_ = grid_rows
        self.cols_ = grid_cols
        self.vertex_ = {}
        self.graph_init()
        self.graph_status_ = accurate_graph
        self.L2_error_total_ = 0.0

        self.ROIs_ = ROIS
        self.nz_ig_zones_ = NZIG

        self.max_ig_ = 0.0
        self.max_ig_vertex_ = []

        if not accurate_graph:
            self.gpr_ = None
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
            vert = Vertex(idx,coord_x,coord_y,0.5,0.0)
            # print("Vertex {}, coordinate is ({}, {})".format(idx, coord_x,coord_y))
            self.vertex_[idx] = vert
    
    def obs_init(self):
        pass
        # for ob_id in obs:
        #     obs_vt = self.vertex_[ob_id]
        #     obs_vt.p_ = 1.0
        #     obs_vt.occupancy_ = "OCCUPIED"

    def set_vertex(self, id, occ):
        vert = self.vertex_[id]
        if occ == "FREE":
            vert.p_ = 0.0
            vert.occupancy_ = "FREE"
        elif occ == "OCCUPIED":
            vert.p_ = 1.0
            vert.occupancy_ = "OCCUPIED"

    def graph_vis(self,type="Mean"):
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
        # plt.show()
    
    def graph_data_handler(self, channel, data):
        msg = map_data.decode(data)
        for cell in msg.vertex_:
            idx = cell.idx_
            self.vertex_[idx].ig_ = cell.ig_
            self.vertex_[idx].p_ = cell.p_
            if self.vertex_[idx].p_ == 0.0:
                self.vertex_[idx].occupancy_ = "FREE"
            elif self.vertex_[idx].p_ == 1.0:
                self.vertex_[idx].occupancy_ = "OCCUPIED"
        
        self.L2_error_total_ = 0.0
        self.max_ig_ = 0.0
        self.max_ig_vertex_ = []
        self.bayesian_opt_ = 0.0
        self.bayesian_relative_err_ = 0.0

    def adjacent_prob(self,true_graph):
        for idx in self.vertex_.keys():
            vert = true_graph.vertex_[idx]        
            x = vert.coordinate_[1]
            y = vert.coordinate_[0]
            if idx in ROIs:
                neigh_p = [vert.p_]
            else:
                neigh_p = [INT_MAX]

            for d in dir:
                new_x = x+d[0]
                new_y = y+d[1]
                nb_idx = (self.rows_-new_y-1)*self.cols_+new_x
                if 0 <= new_x and new_x < self.cols_ and 0<= new_y and new_y < self.rows_ and nb_idx in ROIs:
                    neigh_p.append(true_graph.vertex_[nb_idx].p_)
                elif 0 <= new_x and new_x < self.cols_ and 0<= new_y and new_y < self.rows_:
                    neigh_p.append(INT_MAX)
                else:
                    neigh_p.append(INT_MIN)
                neigh_p.sort()
            self.vertex_[idx].probs_ = neigh_p

    def training_samples(self,samples_idx,true_graph):
        for idx in samples_idx:
            if self.vertex_[idx].probs_ not in self.training_x_:
                # print("The index is {}".format(idx))
                self.training_x_.append(self.vertex_[idx].probs_)
                # print("The training sample is {}".format(self.training_x_[-1]))
                self.training_y_.append(true_graph.vertex_[idx].ig_)
                # print("The training y is {}".format(self.training_y_[-1]))
    
    def gp(self,true_graph, samples_idx):
        rbf = 1.0 * RBF(length_scale=1.0)  
        matern = 1.0 * Matern(length_scale=1.0, length_scale_bounds=(1e-1, 10.0),
                        nu=1.5)
        gp_opt = GaussianProcessRegressor(kernel=rbf)
        gp_opt.fit(self.training_x_,self.training_y_)
        self.gpr_ = gp_opt
        # print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))
        # print("Log Marginal Likelihood (optimized): %.3f"
        #     % gp_opt.log_marginal_likelihood(gp_opt.kernel_.theta))

        # Contour 3d
        x1_ = [i for i in range(self.rows_+1)]
        x2_ = [j for j in range(self.cols_+1)]
        X1_, X2_ = np.meshgrid(x1_,x2_)

        y_mean = np.empty([self.rows_+1,self.cols_+1])
        y_true = np.empty([self.rows_+1,self.cols_+1])
        y_std = np.empty([self.rows_+1,self.cols_+1])
        y_mean_u = np.empty([self.rows_+1,self.cols_+1])
        y_mean_d = np.empty([self.rows_+1,self.cols_+1])
        for i in range(self.rows_+1):
            for j in range(self.cols_+1):
                cur_idx = (self.rows_-X1_[i][j]-1)*self.cols_+X2_[i][j]
                # print("X: {}, Y: {}, idx: {}".format(X2_[i][j], X1_[i][j],cur_idx))
                if X1_[i][j]<self.rows_ and X1_[i][j]>=0 and X2_[i][j]<self.cols_ and X2_[i][j]>=0 and cur_idx in nz_ig:
                    y_mean[X2_[i][j],X1_[i][j]], y_std[X2_[i][j],X1_[i][j]] = gp_opt.predict([self.vertex_[cur_idx].probs_], return_std=True)
                    # print("Prediction ========================")
                    # print("Vertex {}, X:{}, Y:{}".format(cur_idx,X2_[i][j],X1_[i][j]))
                    # print("Testing data is {}".format(self.vertex_[cur_idx].probs_))
                    # print("Predicted IG {}".format(y_mean[X2_[i][j],X1_[i][j]]))
                else:
                    y_mean[X2_[i][j] ,X1_[i][j]] = 0
                    y_std[X2_[i][j],X1_[i][j]] = 0.0
                
                y_mean_u[X2_[i][j],X1_[i][j]] = y_mean[X2_[i][j],X1_[i][j]] + y_std[X2_[i][j],X1_[i][j]]
                y_mean_d[X2_[i][j],X1_[i][j]] = y_mean[X2_[i][j],X1_[i][j]] - y_std[X2_[i][j],X1_[i][j]]

                
                if X2_[i][j]<self.cols_ and X1_[i][j]<self.rows_ and X1_[i][j]>=0 and X2_[i][j]>=0:
                    idx_ = (self.rows_-X1_[i][j]-1)*self.cols_+X2_[i][j]
                    y_true[X2_[i][j],X1_[i][j]] = true_graph.vertex_[idx_].ig_
                    self.vertex_[idx_].ig_ = y_mean[X2_[i][j],X1_[i][j]]
                    true_graph.vertex_[idx_].L2_error_ = (true_graph.vertex_[idx_].ig_ - self.vertex_[idx_].ig_)**2
               
                    self.vertex_[idx_].ig_ub_ = y_mean[X2_[i][j],X1_[i][j]] + y_std[X2_[i][j],X1_[i][j]]
                    self.vertex_[idx_].ig_lb_ = y_mean[X2_[i][j],X1_[i][j]] - y_std[X2_[i][j],X1_[i][j]]
                    if round(true_graph.vertex_[idx_].ig_,3) <= round(self.vertex_[idx_].ig_ub_,3) and round(true_graph.vertex_[idx_].ig_,3) >= round(self.vertex_[idx_].ig_lb_,3):
                        self.vertex_[idx_].ig_pred_ = True
                    # print("Idx {}, Coordinate {}, {}, IG is {}".format(idx_,X2_[i][j],X1_[i][j],self.vertex_[idx_].ig_)
            
        # print("The final L2 error is {}".format(np.square(L2_error_sum_)))
        # print("The value is {}".format(plus_sum))
        # self.graph_vis()
        # self.graph_vis("MeanUpBound")
        # self.graph_vis("MeanLowBound")
        # self.graph_vis("InRange")

        # # plt.figure()
        # plt.subplots_adjust(left=0.03,right=1.0,wspace=0.02)
        # plt.subplot(1,2,1)
        # plt.title("IG prediction -- mean")
        # CMAP = plt.get_cmap('coolwarm')
        # plt.pcolormesh(X2_,X1_,y_mean,cmap=CMAP)
        # cb = plt.colorbar(shrink = 1.0)
        # plt.subplot(1,2,2)
        # plt.title("True IG")
        # CMAP = plt.get_cmap('coolwarm')
        # plt.pcolormesh(X2_,X1_,y_true,cmap=CMAP)
        # cb = plt.colorbar(shrink = 1.0)  

        # # plt.figure()
        # plt.title("Diviation")
        # plt.pcolormesh(X2_,X1_,y_std,cmap=CMAP)
        # cb = plt.colorbar(shrink = 1.0)
        # # plt.show()
        

        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.plot_surface(X1_,X2_,y_mean,cmap=cm.coolwarm)
        # ax.plot_surface(X1_,X2_,y_mean_u,color='r')
        # ax.plot_surface(X1_,X2_,y_mean_d,color='b')
        # for id in true_graph.vertex_.keys():
        #     ax.scatter(true_graph.vertex_[id].coordinate_[1],true_graph.vertex_[id].coordinate_[0],true_graph.vertex_[id].ig_,'bo')

    def calculate_L2_error(self):
        self.L2_error_total_ = 0.0
        for vt in self.vertex_.keys():
            self.L2_error_total_ += self.vertex_[vt].L2_error_
        self.L2_error_total_ = np.square(self.L2_error_total_)
        print("The L2 norm error is {}".format(self.L2_error_total_))

    def calculate_ave_L2_norm(self):
        self.calculate_L2_error()
        self.ave_L2_norm_ = self.L2_error_total_/(NUM_NZ_IG)

    def expected_improvement(self,X,xi=0.01):
        mu, sigma = self.gpr_.predict(X, return_std=True)
        mu_sample = self.gpr_.predict(self.training_x_)

        sigma = sigma.reshape(-1, 1)
        
        # Needed for noise-based model,
        # otherwise use np.max(Y_sample).
        # See also section 2.4 in [...]
        mu_sample_opt = np.max(mu_sample)

        with np.errstate(divide='warn'):
            imp = mu - mu_sample_opt - xi
            Z = imp / sigma
            ei = imp * norm.cdf(Z) + sigma * norm.pdf(Z)
            ei[sigma == 0.0] = 0.0
        return ei

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
        dim = len(self.training_x_[0])
        min_val = np.inf
        min_x = None
        
        def min_obj(X):
            # Minimization objective is the negative acquisition function
            return -acquisition(X.reshape(-1, dim))

        def eq_constraint(x):
            return (x-0.0)*(x-0.5)*(x-0.8)*(x-0.2)*(x-INT_MAX)*(x-INT_MIN)*(x-0.3)*(x-0.4)
        
        # Find the best optimum by starting from n_restart different random points.
        random_samples = []
        iteration = 0
        while len(random_samples) < MAX_SAMPLES and iteration < MAX_ITER:
            sample = []
            iteration = iteration+1
            while len(sample) < dim:
                idx = random.randint(0,len(self.poss_probs_)-1)
                sample.append(self.poss_probs_[idx])
            sample.sort()
            flag_ = self.prob_valid(sample)
            if flag_ and sample not in random_samples:
                random_samples.append(sample)

        # print("The random generated samples are {}".format(random_samples))
        for x0 in random_samples:
            cons = ({'type':'eq','fun':eq_constraint})
            res = minimize(min_obj, x0=x0, constraints=cons)       
            if res.fun < min_val:
                min_val = res.fun
                min_x = res.x           
                
        return min_x.reshape(-1, 1)

    def prob_valid(self,sample_p):
        for v_idx in self.vertex_.keys():
            if v_idx in self.ROIs_ and self.vertex_[v_idx].probs_ == sample_p:
                return True
        return False
    
    def complexity_prob(self):
        self.poss_probs_ = []
        for idx in self.ROIs_:
            if self.vertex_[idx].p_ not in self.poss_probs_:
                self.poss_probs_.append(self.vertex_[idx].p_)
        self.poss_probs_.append(INT_MAX)
        self.poss_probs_.append(INT_MIN)
        print("The possible complexity of probability is {}".format(self.poss_probs_))
    
    def duplicate_true_graph(self,true_graph):
        for idx in self.vertex_.keys():
            self.vertex_[idx].p_ = true_graph.vertex_[idx].p_
            self.vertex_[idx].ig_ = true_graph.vertex_[idx].ig_

    def locate_vertex(self,p,true_graph):
        for idx in self.vertex_.keys():
            if self.vertex_[idx].probs_ == p:
                print("The located vertex is {}".format(idx))
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

            
        
def main():
    lc = lcm.LCM()
    num_cases = 100
    case_idx = 0
    true_graph = Grid(True,ROIs,nz_ig)
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


