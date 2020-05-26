import lcm
import matplotlib.pyplot as plt
import numpy as np
import csv

from graph_data import vertex_data
from graph_data import map_data
from graph_data import sensors_data

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

        self.graph_init()

    def graph_reset(self):
        self.vertex_ = {}
        
    def graph_init(self):
        self.graph_reset()

        for idx in range(self.num_col_ * self.num_row_):
            row = self.num_row_ - idx/ self.num_row_ - 1
            col = idx % self.num_col_
            p = 0.5
            ig = 0
            occupancy = "UNKNOWN"
            new_vertex = Vertex(idx, row, col, p, ig, occupancy)
            self.vertex_[idx] = new_vertex

    def map_data_handler(self, channel, data):
        self.graph_init()
        msg = map_data.decode(data)
        for cell in msg.vertex_:
            idx = cell.idx_
            self.vertex_[idx].ig_ = cell.ig_
            self.vertex_[idx].p_ = cell.p_
            if self.vertex_[idx].p_ == 0.0:
                self.vertex_[idx].occupancy_ = "FREE"
            elif self.vertex_[idx].p_ == 1.0:
                self.vertex_[idx].occupancy_ = "OCCUPIED"


    def training_data_handler(self,channel,data):
        msg = sensors_data.decode(data)
        for pos in msg.sensor_pos_:
            self.training_data_.append(pos)

        self.GP_map_vis()

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
    lc = lcm.LCM()
    map = map_vis()

    for i in range(100000):
        Graph_data = str(i) + "GraphDataGP"
        Training_data = str(i) + "TrainingData"
        subscription = lc.subscribe(Graph_data, map.map_data_handler)
        subscription = lc.subscribe(Training_data, map.training_data_handler)
      
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)


if __name__ == '__main__':
    main()