import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import LogNorm
import lcm
import random

from graph_data import vertex_data
from graph_data import map_data

from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic,
                                              ExpSineSquared, DotProduct,
                                              ConstantKernel,WhiteKernel)

# This file is used to generate the plot: L2 error VS number of training data
# The none zero information gain regions contrains 69 cells

grid_rows = 10
grid_cols = 10
obs = [5,6,7,16,32,33,43,52,53,58,61,62,63,64,67,68,69,72,73,78]
training_idx = [3,4,13,14,7,8,17,18,56,57,66,67,72,73,82,83]
# training_idx1 = [6,7,26,27,8,9,28,29,46,47,66,67,48,49,68,69]
# training_idx2 = [14,15,34,35,16,17,36,37,54,55,74,75,56,57,76,77]
# training_idx3 = [212,213,232,233,214,215,234,235,252,253,272,273,254,255,274,275]
# training_idx4 = [284,285,304,305,286,287,306,307,324,325,344,345,326,327,346,347]
# training_idx = training_idx1+training_idx2+training_idx3+training_idx4
dir = [[-1,0],[1,0],[0,-1],[0,1]]
INT_MIN = -10
INT_MAX = -INT_MIN
# Define the ROIs
# 10*10 map
ROIs = []
boundary = [[4,10],[13,20],[23,26],[34,36],[44,48],[52,58],[62,66],[70,76],[80,84],[90,94]]
nz_ig = []
nz_boundary = [[3,10],[12,20],[22,30],[33,38],[42,49],[51,59],[60,68],[70,77],[80,86],[90,95]]
# 20*20 map
# ROIs = []
# boundary = [[8,20],[28,40],[46,60],[66,80],[86,92],[106,112],
#         [128,132],[148,152],[168,176],[188,196],[204,216],[224,236],[244,252],[264,272],[280,292],
#         [300,312],[320,328],[340,348],[360,368],[380,388]]
# nz_ig = []
# nz_boundary = [[7,20],[26,40],[45,60],[65,80],[85,100],[105,113],
#         [126,133],[147,156],[167,177],[184,197],[203,217],[223,237],[243,256],[260,273],[280,293],
#         [300,313],[320,332],[340,349],[360,369],[380,389]]
for r in nz_boundary:
    for id in range(r[0],r[1],1):
        nz_ig.append(id)
for q in boundary:
    for idq in range(q[0],q[1],1):
        ROIs.append(idq)

class Vertex(object):
    def __init__(self,id,pos_x,pos_y,p,ig):
        self.idx_ = id
        self.coordinate_ = [pos_x,pos_y] # number of row, col
        self.p_ = p
        self.occupancy_ = "UNKNOWN"
        self.ig_ = ig
        self.ig_ub_ = ig
        self.ig_lb_ = ig
        self.ig_pred_ = False
        self.L2_error_ = 0.0

class Grid(object):
    def __init__(self,accurate_graph=False):
        self.rows_ = grid_rows
        self.cols_ = grid_cols
        self.vertex_ = {}
        self.graph_init()
        self.graph_status_ = accurate_graph
        self.L2_error_total_ = 0.0
        if self.graph_status_:
            self.obs_init()

    def graph_init(self):
        for idx in range(self.rows_*self.cols_):
            coord_x = self.rows_ - int(idx/self.rows_) - 1
            coord_y = idx%self.rows_
            vert = Vertex(idx,coord_x,coord_y,0.5,0.0)
            # print("Vertex {}, coordinate is ({}, {})".format(idx, coord_x,coord_y))
            self.vertex_[idx] = vert
    
    def obs_init(self):
        for ob_id in obs:
            obs_vt = self.vertex_[ob_id]
            obs_vt.p_ = 1.0
            obs_vt.occupancy_ = "OCCUPIED"

    def set_vertex(self, id, occ):
        vert = self.vertex_[id]
        if occ == "FREE":
            vert.p_ = 0.0
            vert.occupancy_ = "FREE"
        elif occ == "OCCUPIED":
            vert.p_ = 1.0
            vert.occupancy_ = "OCCUPIED"

    def graph_vis(self,type="Mean"):
        plt.figure()
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

        # self.graph_vis("Origin")
        # self.graph_vis()
        # unknown_graph = Grid(False)
        # unknown_graph.gp(self)
        # self.calculate_L2_error()
    
    def adjacent_p(self,idx,true_graph):
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
        return neigh_p

    def gp(self,true_graph, training_idx):
        training_X = []
        training_Y = []
        for idx in training_idx:
            training_Y.append(true_graph.vertex_[idx].ig_)
            nbs_p = self.adjacent_p(idx,true_graph)
            print("The vert {}".format(idx))
            print("The training X is {}".format(nbs_p))
            print("The training Y is {}".format(training_Y[-1]))
            print("===============================================")
            training_X.append(nbs_p)

        rbf = 1.0 * RBF(length_scale=1.0)  
        matern = 1.0 * Matern(length_scale=1.0, length_scale_bounds=(1e-1, 10.0),
                        nu=1.5)
        gp_opt = GaussianProcessRegressor(kernel=rbf)
        gp_opt.fit(training_X,training_Y)
        print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))
        print("Log Marginal Likelihood (optimized): %.3f"
            % gp_opt.log_marginal_likelihood(gp_opt.kernel_.theta))

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
                print("X: {}, Y: {}, idx: {}".format(X2_[i][j], X1_[i][j],cur_idx))
                if X1_[i][j]<self.rows_ and X1_[i][j]>=0 and X2_[i][j]<self.cols_ and X2_[i][j]>=0 and cur_idx in nz_ig:
                    neigh_p = self.adjacent_p(cur_idx,true_graph)
                    y_mean[X2_[i][j],X1_[i][j]], y_std[X2_[i][j],X1_[i][j]] = gp_opt.predict([neigh_p], return_std=True)
                    print("Prediction ========================")
                    print("Vertex {}, X:{}, Y:{}".format(cur_idx,X2_[i][j],X1_[i][j]))
                    print("Testing data is {}".format(neigh_p))
                    print("Predicted IG {}".format(y_mean[X2_[i][j],X1_[i][j]]))
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
        self.graph_vis()
        self.graph_vis("MeanUpBound")
        self.graph_vis("MeanLowBound")
        self.graph_vis("InRange")

        plt.figure()
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

        plt.figure()
        plt.title("Diviation")
        plt.pcolormesh(X2_,X1_,y_std,cmap=CMAP)
        cb = plt.colorbar(shrink = 1.0)
        plt.show()

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
        

def main():
    lc = lcm.LCM()
    true_graph = Grid(True)
    Graph_data = "1GraphData"
    subscription = lc.subscribe(Graph_data, true_graph.graph_data_handler)
    try:
        if True:
            lc.handle()
            true_graph.graph_vis("Origin")
            true_graph.graph_vis()
            unknown_graph = Grid(False)
            unknown_graph.gp(true_graph, training_idx)
            true_graph.calculate_L2_error()

    
    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)


if __name__ == '__main__':
    main()

