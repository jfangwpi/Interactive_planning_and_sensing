import numpy as np
import math
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sklearn.metrics.classification import accuracy_score, log_loss
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process.kernels import RBF
from sklearn.gaussian_process.kernels import Matern
# This file is used to visualize the gaussian process occupancy grid map (Work with test_gpc_sensor.cpp)
fontsize_dis = 8

num_row = 30
num_col = 30

class Vertex(object):
    def __init__(self, index, pos_x, pos_y, p, ig, occu):
        self.idx_ = index
        self.pos_ = [pos_x, pos_y]
        self.p_ = p
        self.ig_ = ig
        self.occupancy_ = occu

class map_vis(object):
    def __init__(self):
        self.num_row_ = num_row
        self.num_col_ = num_row
        self.num_fig = 0
        self.training_data_ = []
        self.vertex_ = {}

        self.graph_init()

    def graph_reset(self):
        self.vertex_ = {}
        
    def graph_init(self):
        self.graph_reset()

        for idx in range(self.num_col_ * self.num_row_):
            row = self.num_row_ - int(idx/ self.num_row_) - 1
            col = idx % self.num_col_
            # print("cell {}, ({},{})".format(idx,row,col))
            p = 0.5
            ig = 0
            occupancy = "UNKNOWN"
            new_vertex = Vertex(idx, row, col, p, ig, occupancy)
            self.vertex_[idx] = new_vertex

    def vis(self):
        # Draw grid map 
        x_min = 0
        x_max = self.num_col_
        y_min = 0
        y_max = self.num_row_
        plt.grid(color='black', linestyle='-', linewidth=1)
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.xticks(np.arange(x_min, x_max+1, 1))
        plt.yticks(np.arange(y_min, y_max+1, 1))

        # Draw each cell
        for cell in self.vertex_.values():
            plt.text(cell.pos_[1] + 0.25, cell.pos_[0] + 0.25,"{}".format(round(cell.p_,3)), color='black', fontsize=20)
        
        ZZ = np.empty([self.num_col_, self.num_row_])
        for cell in self.vertex_.values():
            ZZ[cell.pos_[1],cell.pos_[0]] = cell.p_

        # The number here is the num of col and row + 1
        XX, YY = np.mgrid[0:self.num_col_:11j, 0:self.num_row_:11j]

        CMAP = plt.get_cmap('jet')
        plt.pcolormesh(XX,YY,ZZ,cmap=CMAP)
        cb = plt.colorbar(shrink = 1.0)
        cb.ax.set_yticklabels(cb.ax.get_yticklabels(), fontsize=30)
    

    def GP_map_vis(self):
        x_min = 0
        x_max = self.num_col_
        y_min = 0
        y_max = self.num_row_
        plt.grid(color='black', linestyle='-', linewidth=1.5)
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.xticks(np.arange(x_min, x_max+1, 10))
        plt.yticks(np.arange(y_min, y_max+1, 10))

         # Draw each cell
        for cell in self.vertex_.values():
            plt.text(cell.pos_[1] + 0.25, cell.pos_[0] + 0.25,"{}".format(round(cell.p_,2)), color='black', fontsize=fontsize_dis)
            if (cell.p_ >= 0.45 and cell.p_ <= 0.55):
                x_region = np.arange(cell.pos_[1], cell.pos_[1] + 2, 1)
                y_region = cell.pos_[0]
                plt.fill_between(x_region, y_region, y_region +1, facecolor='gray', interpolate=True)
                
            if (cell.p_ > 0.55):
                x_region = np.arange(cell.pos_[1], cell.pos_[1] + 2, 1)
                y_region = cell.pos_[0]
                plt.fill_between(x_region, y_region, y_region +1, facecolor='black', interpolate=True)
        
        for dt in self.training_data_:
            cv = self.vertex_[dt]
            row = cv.pos_[1] + 0.5
            col = cv.pos_[0] + 0.5
            plt.plot(row,col,"*r",markersize=20)

        plt.show()


def main():
    map = map_vis()
    # define the obstacle
    obs1_idx = np.arange(18,27,1.0)
    obs1 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs1_idx]
    obs2_idx = np.arange(48,57,1.0)
    obs2 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs2_idx]
    obs3_idx = np.arange(78,87,1.0)
    obs3 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs3_idx]
    obs4_idx = np.arange(108,117,1.0)
    obs4 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs4_idx]
    obs5_idx = np.arange(138,147,1.0)
    obs5 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs5_idx]
    obs6_idx = np.arange(168,177,1.0)
    obs6 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs6_idx]
    block1 = obs1 + obs2 + obs3 + obs4 + obs5 + obs6

    obs7_idx = np.arange(180,188,1.0)
    obs7 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs7_idx]
    obs8_idx = np.arange(210,218,1.0)
    obs8 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs8_idx]
    obs9_idx = np.arange(240,248,1.0)
    obs9 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs9_idx]
    obs10_idx = np.arange(270,278,1.0)
    obs10 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs10_idx]
    obs11_idx = np.arange(300,308,1.0)
    obs11 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs11_idx]
    obs12_idx = np.arange(330,338,1.0)
    obs12 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs12_idx]
    obs13_idx = np.arange(360,368,1.0)
    obs13 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs13_idx]
    obs14_idx = np.arange(390,398,1.0)
    obs14 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs14_idx]
    obs15_idx = np.arange(420,428,1.0)
    obs15 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs15_idx]
    obs16_idx = np.arange(450,458,1.0)
    obs16 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs16_idx]
    obs17_idx = np.arange(480,488,1.0)
    obs17 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs17_idx]
    block2 = obs7 + obs8 + obs9 + obs10 + obs11 + obs12 + obs13 + obs14 + obs15 + obs16 + obs17

    obs18_idx = np.arange(285,300,1.0)
    obs18 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs18_idx]
    obs19_idx = np.arange(315,330,1.0)
    obs19 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs19_idx]
    obs20_idx = np.arange(345,360,1.0)
    obs20 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs20_idx]
    obs21_idx = np.arange(375,390,1.0)
    obs21 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs21_idx]
    obs22_idx = np.arange(417,420,1.0)
    obs22 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs22_idx]
    obs23_idx = np.arange(447,450,1.0)
    obs23 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs23_idx]
    obs24_idx = np.arange(477,480,1.0)
    obs24 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs24_idx]
    obs25_idx = np.arange(507,510,1.0)
    obs25 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs25_idx]
    obs26_idx = np.arange(537,540,1.0)
    obs26 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs26_idx]
    block3 = obs18 + obs19 + obs20 + obs21 + obs22 + obs23 + obs24 + obs25 + obs26

    obs27_idx = np.arange(600,605,1.0)
    obs27 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs27_idx]
    obs28_idx = np.arange(630,635,1.0)
    obs28 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs28_idx]
    obs29_idx = np.arange(660,665,1.0)
    obs29 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs29_idx]
    obs30_idx = np.arange(690,695,1.0)
    obs30 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs30_idx]
    obs31_idx = np.arange(720,725,1.0)
    obs31 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs31_idx]
    block4 = obs27 + obs28 + obs29 + obs30 + obs31

    obs32_idx = np.arange(729,740,1.0)
    obs32 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs32_idx]
    obs33_idx = np.arange(759,770,1.0)
    obs33 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs33_idx]
    obs34_idx = np.arange(789,800,1.0)
    obs34 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs34_idx]
    obs35_idx = np.arange(819,830,1.0)
    obs35 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs35_idx]
    obs36_idx = np.arange(848,860,1.0)
    obs36 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs36_idx]
    obs37_idx = np.arange(879,890,1.0)
    obs37 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs37_idx]
    block5 = obs32 + obs33 + obs34 + obs35 + obs36 + obs37

    obs38_idx = np.arange(713,718,1.0)
    obs38 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs38_idx]
    obs39_idx = np.arange(743,748,1.0)
    obs39 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs39_idx]
    obs40_idx = np.arange(773,778,1.0)
    obs40 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs40_idx]
    obs41_idx = np.arange(803,808,1.0)
    obs41 = [[map.vertex_[id].pos_[0],map.vertex_[id].pos_[1]] for id in obs41_idx]
    block6 = obs38 + obs39 + obs40 + obs41
    
    obs_ = block1 + block2 + block3 + block4 + block5 + block6
    print("the length of obs is {}".format(len(obs_)))


    # Define the sensors position
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
    sensors_17 = [338,444,310,872,512]
    sensors_18 = [257,226,159,413,446]
    sensors_19 = [839,696,700,77,224]
    sensors_20 = [399,669,32,281,1]
    sensors_21 = [840,506,535,536,475]

    sensors = sensors_1 + sensors_2 + sensors_3 + sensors_4 + sensors_5 + sensors_6 + sensors_7 + sensors_8 + sensors_9 + sensors_10
    sensors = sensors + sensors_11 + sensors_12 + sensors_13 + sensors_14 + sensors_15 + sensors_16 + sensors_17 + sensors_18 + sensors_19 + sensors_20 + sensors_21
    
    sensors_X = []
    for ss in sensors:
        sensors_X.append([map.vertex_[ss].pos_[0],map.vertex_[ss].pos_[1]])

    print("The number of sensors are {}".format(len(sensors_X)))

    # Build the training data
    training_X = []
    training_Y = []
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
        if n_y < num_row:
            if [s_x,n_y] not in training_X:
                training_X.append([s_x,n_y])
                if [s_x,n_y] not in obs_:
                    training_Y.append(0)
                else:
                    training_Y.append(1)

        # Down
        n_y = s_y - 1
        if n_y >= 0:
            if [s_x, n_y] not in training_X:
                training_X.append([s_x,n_y])
                if [s_x,n_y] not in obs_:
                    training_Y.append(0)
                else:
                    training_Y.append(1)
        
        # Left
        n_x = s_x - 1
        if n_x >= 0:
            if [n_x,s_y] not in training_X:
                training_X.append([n_x,s_y])
                if [n_x,s_y] not in obs_:
                    training_Y.append(0)
                else:
                    training_Y.append(1)

        # Right
        n_x = s_x + 1 
        if n_x < num_col:
            if [n_x,s_y] not in training_X:
                training_X.append([n_x,s_y])
                if [n_x,s_y] not in obs_:
                    training_Y.append(0)
                else:
                    training_Y.append(1)
    
    interested_idx = [0,17,29,125,199,284,398,409,476,510,701,684,870,899]
    for i_idx in interested_idx:
        coord = [map.vertex_[i_idx].pos_[0],map.vertex_[i_idx].pos_[1]]
        if coord not in training_X:
            training_X.append(coord)
            training_Y.append(0)

    print("The number of training data is {}".format(len(training_X)))

    gp_opt = GaussianProcessClassifier(RBF(length_scale=1.0))
    gp_opt.fit(training_X,training_Y)
    print("The trained hyperparameter are {}".format((gp_opt.kernel_.theta)))   

    # Predict the probability of vertex
    for v in map.vertex_.values():
        if v.pos_ not in training_X:
            coord = [v.pos_[0],v.pos_[1]]
            p_occ = gp_opt.predict_proba(np.reshape(coord,(-1,2)))[:,1]
            occ_ = gp_opt.predict(np.reshape(coord,(-1,2)))

            v.p_ = p_occ[0]
            if v.p_ >= 0.45 and v.p_ <= 0.55:
                v.occupancy_ = "UNKNOWN"
            elif v.p_ > 0.55:
                v.occupancy_ = "OCCUPIED"
            else:
                v.occupancy_ = "FREE"
        else:
            e_idx = training_X.index(v.pos_)
            if training_Y[e_idx] == 1:
                v.p_ = 1.0
                v.occupancy_ = "OCCUPIED"
            else:
                v.p_ = 0.0
                v.occupancy_ = "FREE"
    
    map.GP_map_vis()
  


if __name__ == '__main__':
    main()