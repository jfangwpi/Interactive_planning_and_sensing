import lcm
import matplotlib.pyplot as plt
import matplotlib.pylab as pl
from matplotlib.colors import ListedColormap
import numpy as np
import csv

from graph_data import vertex_data
from graph_data import map_data
from graph_data import path_data
from graph_data import paths_data
from graph_data import entropy_trend_data
from graph_data import range_checked_data
from graph_data import entropy_path_trend_data
from graph_data import entropy_paths_trend_data
from graph_data import sensors_data
from graph_data import local_hspts_data


fontsize_dis = 50
style_dis = "split"

num_row = 30
num_col = 30
# num_row = 10
# num_col = 10
# num_row = 5
# num_col = 5
# num_row = 20
# num_col = 20

# agents = {1:0,2:29,3:870,4:899}
# agents = {1:0,2:99}
# agents = {1:0}
# agents = {1:0,2:380,3:399}
agents = {1:0,2:29,3:870,4:899}

# tasks = {3:125, 4:199, 5:17, 6:398, 7:510, 8:409, 9:476, 10:701, 11:684, 12:284}
# tasks = {1:67,2:76,3:139,4:180,5:215,6:309}
# tasks = {1:50,2:95,3:7}
# tasks = {1:24}
tasks = {3:880,4:457,5:194,6:108,7:145,8:356,9:290,10:505,11:565,12:865}

class Vertex(object):
    def __init__(self, index, pos_x, pos_y, p, ig, occu):
        self.idx_ = index
        self.pos_ = [pos_x, pos_y]
        self.p_ = p
        self.ig_ = ig
        self.occupancy_ = occu

class Sample(object):
    def __init__(self, sample, p):
        self.sample_ = sample
        self.p_ = p


class map_vis(object):
    def __init__(self):
        self.num_row_ = num_row
        self.num_col_ = num_row
        self.path_collection_ = {}
        self.num_fig = 0
        self.entropy_paths_ = {}
        self.sensors_ = []
        self.hspts_ = []

        self.tasks_ = {}
        self.agents_ = {}

        self.graph_init()

    def graph_reset(self):
        self.vertex_ = {}
        self.num_agents = 0
        self.agents_ = {}
        self.path_collection_ = {}
        self.entropy_paths_ = {}
        self.sensors_ = []
        self.hspts_ = []
        self.tasks_ = {}


    def graph_init(self):
        self.graph_reset()

        for idx in range(self.num_col_ * self.num_row_):
            row = self.num_row_ - idx//self.num_row_ - 1
            col = idx % self.num_col_
            p = 0.5
            ig = 0
            occupancy = "UNKNOWN"
            new_vertex = Vertex(idx, row, col, p, ig, occupancy)
            self.vertex_[idx] = new_vertex

        for ag in agents.keys():
            self.agents_[ag] = agents[ag]

        for tsk in tasks.keys():
            self.tasks_[tsk] = tasks[tsk]


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

    def hspts_data_handler(self, channel, data):
        msg = local_hspts_data.decode(data)
        for pos in msg.hspts_:
            self.hspts_.append(pos)


    def sensors_data_handler(self, channel, data):
        msg = sensors_data.decode(data)
        for pos in msg.sensor_pos_:
            self.sensors_.append(pos)

        self.visualization()


    def paths_data_handler(self, channel, data):
        msg = paths_data.decode(data)
        for path_idx in range(msg.num_path_):
            path_ = []
            if msg.path_collection_[path_idx].cell_ == []:
                continue
            else:
                for cell in msg.path_collection_[path_idx].cell_:
                    path_.append(cell)
            self.path_collection_[path_idx] = path_

        # self.visualization()


    def entropy_trend_data_handler(self, channel, data):
        msg = entropy_trend_data.decode(data)
        self.entropy_trend_ = []
        with open("Entropy_trend.cvs", mode="w") as csv_file:
            fieldnames = ["iter_t", "entropy"]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for c, entropy in enumerate(msg.entropy_, 1):
                self.entropy_trend_.append(entropy)
                writer.writerow({"iter_t": c, "entropy": entropy})

            
    def range_trend_data_handler(self, channel, data):
        msg = range_checked_data.decode(data)
        self.range_checked_ = []
        with open("Range_trend.cvs", mode="w") as csv_file:
            fieldnames = ["iter_t", "range"]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for c, r in enumerate(msg.range_):
                self.range_checked_.append(r)
                writer.writerow({"iter_t": c, "range": r})


    def entropy_trend_paths_data_handler(self, channel, data):
        msg = entropy_paths_trend_data.decode(data)
        self.entropy_paths_ = {}
        for r in msg.entropy_paths_:
            self.entropy_paths_[r.agent_idx_] = [en for en in r.entropy_path_]
                
        self.trend_vis()

        with open("Entropy_trend_paths.cvs", mode="w") as csv_file:
            fieldnames = ["iter_t", "entropy1", "entropy2", "entropy3"]
            # fieldnames = ["iter_t", "entropy1"]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader() 
            for c in range(len(self.entropy_paths_[0])):
                # writer.writerow({"iter_t": c, "entropy1": self.entropy_paths_[0][c]})
                writer.writerow({"iter_t": c, "entropy1": self.entropy_paths_[0][c], "entropy2":self.entropy_paths_[1][c], "entropy3":self.entropy_paths_[2][c]})

        
    def trend_vis(self):
        if self.range_checked_:
            plt.figure(num=None, figsize= (50,25), dpi=80, facecolor='w',edgecolor='k')
            iterations = [i+1 for i in range(len(self.entropy_trend_))]
            plt.subplot(1,2,1)
            plt.plot(iterations, self.entropy_trend_, 'b-', linewidth=8)
            plt.yticks(fontsize = 30)
            plt.xticks(fontsize = 30)
            plt.xlabel("Num of iteration", fontsize=30)
            plt.ylabel("Entropy of map", fontsize=30)
            plt.ylim(bottom=0)
            plt.xlim(left=1,right=iterations[-1])

            plt.subplot(1,2,2)
            iterations1 = [j+1 for j in range(len(self.range_checked_))]
            plt.plot(iterations1, self.range_checked_, 'r-', linewidth=8)
            plt.yticks(fontsize = 30)
            plt.xticks(fontsize = 30)
            plt.xlabel("Num of iteration", fontsize=30)
            plt.ylabel("Focused Range", fontsize=30)
            plt.ylim(bottom=0)
            plt.xlim(left=1,right=iterations1[-1])
        else:
            plt.figure(num=None, figsize= (25,25), dpi=80, facecolor='w',edgecolor='k')
            iterations = [i+1 for i in range(len(self.entropy_trend_))]
            plt.plot(iterations, self.entropy_trend_, 'b-', linewidth=8)
            plt.yticks(fontsize = 30)
            plt.xticks(fontsize = 30)
            plt.xlabel("Num of iteration", fontsize=30)
            plt.ylabel("Entropy of map", fontsize=30)
            plt.ylim(bottom=0)
            plt.xlim(left=1,right=iterations[-1])

        plt.savefig("Graph_data_trend.png")


        if self.entropy_paths_:
            plt.figure(num=None, figsize=(25,25),dpi=80,facecolor='w',edgecolor='k')
            colors_ = ['yellow', 'blue', 'red', 'green']
            for r, path in enumerate(self.entropy_paths_.values()):
                iterations2 = [i+1 for i in range(len(path))]
                label_ = "Agent " + str(r)
                plt.plot(iterations2, path, colors_[r], linewidth=8, label=label_)
            plt.legend(fontsize = 30)
            plt.yticks(fontsize = 30)
            plt.xticks(fontsize = 30)
            plt.xlabel("Num of iteration", fontsize=30)
            plt.ylabel("Entropy of path", fontsize=30)
            plt.ylim(bottom=0)
            plt.xlim(left=1,right=iterations[-1])

        plt.savefig("Entropy_trend_paths.png")
        plt.close('all')

    def vis(self):
        # Draw grid map 
        x_min = 0
        x_max = self.num_col_
        y_min = 0
        y_max = self.num_row_
        plt.grid(color='black', linestyle='-', linewidth=1)
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.xticks(np.arange(x_min, x_max+1, 10))
        plt.yticks(np.arange(y_min, y_max+1, 10))

        # Draw each cell
        for cell in self.vertex_.values():
            # plt.text(cell.pos_[1] + 0.25, cell.pos_[0] + 0.25,"{}".format(round(cell.p_,3)), color='black', fontsize=fontsize_dis)
            if (cell.occupancy_ == "UNKNOWN"):
                x_region = np.arange(cell.pos_[1], cell.pos_[1] + 2, 1)
                y_region = cell.pos_[0]
                plt.fill_between(x_region, y_region, y_region +1, facecolor='gray', interpolate=True)

            if (cell.occupancy_ == "INTERESTED"):
                x_region = np.arange(cell.pos_[1], cell.pos_[1] + 2, 1)
                y_region = cell.pos_[0]
                plt.fill_between(x_region, y_region, y_region +1, facecolor='yellow', interpolate=True)
                
            if (cell.occupancy_ == "OCCUPIED"):
                x_region = np.arange(cell.pos_[1], cell.pos_[1] + 2, 1)
                y_region = cell.pos_[0]
                plt.fill_between(x_region, y_region, y_region +1, facecolor='black', interpolate=True)

        # Draw agent position 
        for agent_idx in self.agents_.keys():
            row = (self.num_row_ - self.agents_[agent_idx] // self.num_row_ - 1)
            col = self.agents_[agent_idx] % self.num_row_
            plt.text(col + 0.05, row + 0.12,"V {}".format(agent_idx), color='yellow', fontsize=fontsize_dis, fontweight='bold')
            x_region = np.arange(col, col + 2, 1)
            y_region = row
            plt.fill_between(x_region, y_region, y_region +1, facecolor='red', interpolate=True)
        

        for tsk in self.tasks_.keys():
            row = (self.num_row_ - self.tasks_[tsk] // self.num_row_ - 1)
            col = self.tasks_[tsk] % self.num_row_
            plt.text(col + 0.55, row + 0.1,"P{}".format(tsk+1), color='black', fontsize=fontsize_dis, fontweight='bold')
            x_region = np.arange(col, col + 2, 1)
            y_region = row
            plt.fill_between(x_region, y_region, y_region +1, facecolor='yellow', interpolate=True)


        # Draw path
        if self.path_collection_:
            for idx in range(len(self.path_collection_)):
                path = self.path_collection_[idx]

                if path != []: 
                    # Mark Start cell
                    if idx == 4:
                        start_id = path[0]
                        start_vertex = self.vertex_[start_id]
                        plt.text(start_vertex.pos_[1]+0.40, start_vertex.pos_[0]+0.65, 'S1', color='blue', fontsize=fontsize_dis, fontweight='bold')
                    if idx == 5:
                        start_id = path[0]
                        start_vertex = self.vertex_[start_id]
                        plt.text(start_vertex.pos_[1]+0.40, start_vertex.pos_[0]+0.65, 'S2', color='blue', fontsize=fontsize_dis, fontweight='bold')
                    if idx == 6:
                        start_id = path[0]
                        start_vertex = self.vertex_[start_id]
                        plt.text(start_vertex.pos_[1]+0.40, start_vertex.pos_[0]+0.65, 'S3', color='blue', fontsize=fontsize_dis, fontweight='bold')
                    if idx == 7:
                        start_id = path[0]
                        start_vertex = self.vertex_[start_id]
                        plt.text(start_vertex.pos_[1]+0.40, start_vertex.pos_[0]+0.65, 'S4', color='blue', fontsize=fontsize_dis, fontweight='bold')
            

                    # Mark End Cell
                    if idx <= 3:
                        end_id = path[-1]
                        end_vertex = self.vertex_[end_id]
                        plt.text(end_vertex.pos_[1] + 0.1, end_vertex.pos_[0] + 0.1, 'F', color='green', fontsize=fontsize_dis, fontweight='bold')

                    # Draw the path
                    for v_idx in range(len(path) - 1):
                        v1_id = path[v_idx]
                        v1 = self.vertex_[v1_id]
                        v2_id = path[v_idx+1]
                        v2 = self.vertex_[v2_id]
                        # plt.text(v1.pos_[1] + 0.25, v1.pos_[0] + 0.25,"{}".format(round(v1.p_,3)), color='black', fontsize=30)

                        y = np.linspace(v1.pos_[0] + 0.5, v2.pos_[0] + 0.5, 100)
                        x = np.linspace(v1.pos_[1] + 0.5, v2.pos_[1] + 0.5, 100)
                        
                        if idx <= 3:
                            plt.plot(x,y,'y-', linewidth=15)
                        else:
                            plt.plot(x+0.1,y+0.1,'b--', linewidth=15)
                else:
                    continue
    def map_vis(self):
        # Draw grid map 
        x_min = 0
        x_max = self.num_col_
        y_min = 0
        y_max = self.num_row_
        plt.grid(color='black', linestyle='-', linewidth=1)
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.xticks(np.arange(x_min, x_max+1, 10))
        plt.yticks(np.arange(y_min, y_max+1, 10))

        # Draw each cell
        # for cell in self.vertex_.values():
        #     plt.text(cell.pos_[1] + 0.1, cell.pos_[0] + 0.1,"{}".format(round(cell.p_,3)), color='black', fontweight = 'bold', fontsize=fontsize_dis)

        # Draw the heat map
        self.vertex_[0].p_ = 1.0
        ZZ = np.empty([self.num_col_, self.num_row_])
        for cell in self.vertex_.values():
            ZZ[cell.pos_[1],cell.pos_[0]] = cell.p_
        
        self.vertex_[0].p_ = 0.0

        # The number here is the num of col and row + 1
        XX, YY = np.mgrid[0:self.num_col_:31j, 0:self.num_row_:31j]
        
        CMAP = plt.get_cmap('binary')
        plt.pcolormesh(XX,YY,ZZ,cmap=CMAP)
        cb = plt.colorbar(shrink = 1.0)
        cb.ax.set_yticklabels(cb.ax.get_yticklabels(), fontsize=50)

        # Draw agent position 
        for agent_idx in self.agents_.keys():
            row = (self.num_row_ - self.agents_[agent_idx] // self.num_row_ - 1)
            col = self.agents_[agent_idx] % self.num_row_ 
            self.vertex_[self.agents_[agent_idx]].p_ = 1.0
            plt.text(col - 0.1, row + 0.65,"V {}".format(agent_idx), color='yellow', fontsize=fontsize_dis, fontweight='bold')
            x_region = np.arange(col, col + 2, 1)
            y_region = row
            plt.fill_between(x_region, y_region, y_region +1, facecolor='red', interpolate=True)
        

        for tsk in self.tasks_.keys():
            row = (self.num_row_ - self.tasks_[tsk] // self.num_row_ - 1)
            col = self.tasks_[tsk] % self.num_row_
            plt.text(col + 0.55, row + 0.1,"P{}".format(tsk), color='black', fontsize=fontsize_dis, fontweight='bold')
            x_region = np.arange(col, col + 2, 1)
            y_region = row
            plt.fill_between(x_region, y_region, y_region +1, facecolor='yellow', interpolate=True)

        # Draw path
        if self.path_collection_:
            for idx in range(len(self.path_collection_)):
                path = self.path_collection_[idx]

                if path != []: 
                    # Mark Start cell
                    if idx == 4:
                        start_id = path[0]
                        start_vertex = self.vertex_[start_id]
                        plt.text(start_vertex.pos_[1]+0.40, start_vertex.pos_[0]-0.2, 'S1', color='blue', fontsize=fontsize_dis, fontweight='bold')
                    if idx == 5:
                        start_id = path[0]
                        start_vertex = self.vertex_[start_id]
                        plt.text(start_vertex.pos_[1]+0.40, start_vertex.pos_[0]-0.2, 'S2', color='blue', fontsize=fontsize_dis, fontweight='bold')
                    if idx == 6:
                        start_id = path[0]
                        start_vertex = self.vertex_[start_id]
                        plt.text(start_vertex.pos_[1]+0.40, start_vertex.pos_[0]-0.2, 'S3', color='blue', fontsize=fontsize_dis, fontweight='bold')
                    if idx == 7:
                        start_id = path[0]
                        start_vertex = self.vertex_[start_id]
                        plt.text(start_vertex.pos_[1]+0.40, start_vertex.pos_[0]-0.2, 'S4', color='blue', fontsize=fontsize_dis, fontweight='bold')


                    # Mark End Cell
                    if idx <= 3:
                        end_id = path[-1]
                        end_vertex = self.vertex_[end_id]
                        plt.text(end_vertex.pos_[1] - 0.1, end_vertex.pos_[0] + 0.25, 'F', color='green', fontsize=fontsize_dis, fontweight='bold')

                    # Draw the path
                for v_idx in range(len(path) - 1):
                    v1_id = path[v_idx]
                    v1 = self.vertex_[v1_id]
                    v2_id = path[v_idx+1]
                    v2 = self.vertex_[v2_id]
                    # plt.text(v1.pos_[1] + 0.25, v1.pos_[0] + 0.25,"{}".format(round(v1.p_,3)), color='black', fontsize=30)

                    y = np.linspace(v1.pos_[0] + 0.5, v2.pos_[0] + 0.5, 100)
                    x = np.linspace(v1.pos_[1] + 0.5, v2.pos_[1] + 0.5, 100)
                    
                    if idx <= 3:
                        plt.plot(x,y,'y-', linewidth=15)
                    else:
                        plt.plot(x+0.1,y+0.1,'b--', linewidth=15)
                # else:
                #     continue



    def ig_vis(self):
        # Draw grid map 
        x_min = 0
        x_max = self.num_col_
        y_min = 0
        y_max = self.num_row_
        plt.grid(color='black', linestyle='-', linewidth=1)
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.xticks(np.arange(x_min, x_max+1, 10))
        plt.yticks(np.arange(y_min, y_max+1, 10))

        # Draw each cell
        # for cell in self.vertex_.values():
        #     # if cell.idx_ in self.hspts_:
        #     if cell.ig_ < 0: 
        #         cell.ig_ = 0.0
        #     plt.text(cell.pos_[1] + 0.1, cell.pos_[0] + 0.1,"{}".format(round(cell.ig_,3)), color='black', fontweight = 'bold', fontsize=fontsize_dis)

        # Draw the heat map
        ZZ = np.empty([self.num_col_, self.num_row_])
        for cell in self.vertex_.values():
            ZZ[cell.pos_[1],cell.pos_[0]] = cell.ig_

        # The number here is the num of col and row + 1
        XX, YY = np.mgrid[0:self.num_col_:31j, 0:self.num_row_:31j]
        
        cmap = pl.cm.jet
        CMAP = cmap(np.arange(cmap.N))
        # CMAP = plt.get_cmap('coolwarm')
        CMAP[:,-1] = np.linspace(0,1,cmap.N)
        CMAP = ListedColormap(CMAP)
        plt.pcolormesh(XX,YY,ZZ,cmap=CMAP)
        cb = plt.colorbar(shrink = 1.0)
        cb.ax.set_yticklabels(cb.ax.get_yticklabels(), fontsize=50)

    def visualization(self):   
        self.num_fig = self.num_fig + 1

        if style_dis == "combined":
            plt.figure(num=None, figsize=(55, 25), dpi=80, facecolor='w', edgecolor='k')
            plt.subplot(1,2,1)
            self.vis()
            plt.xticks(fontsize = fontsize_dis)
            plt.yticks(fontsize = fontsize_dis)

            plt.subplot(1,2,2)
            self.ig_vis() 
            plt.xticks(fontsize = fontsize_dis)
            plt.yticks(fontsize = fontsize_dis)

            plt.suptitle("Occupancy grid map at iteration t = {}".format(self.num_fig/7 + 1), fontsize = fontsize_dis)

            for sensor in self.sensors_:
                cv = self.vertex_[sensor]
                row = cv.pos_[1] + 0.5
                col = cv.pos_[0] + 0.5
                plt.plot(row,col,"*b",markersize=40)

            plt.savefig("image{}.png".format(self.num_fig))
            print("{} figures have been saved".format(self.num_fig))
            plt.close('all')

        elif style_dis == "split":
            plt.figure(num=None, figsize=(30, 25), dpi=80, facecolor='w', edgecolor='k')
            self.map_vis()
            plt.xticks(fontsize = fontsize_dis)
            plt.yticks(fontsize = fontsize_dis)
            # plt.title("Occupancy grid map at iteration t = {}".format(self.num_fig), fontsize = fontsize_dis)
            plt.savefig("gridmapimage{}.png".format(self.num_fig))

            plt.figure(num=None, figsize=(30, 25), dpi=80, facecolor='w', edgecolor='k')
            self.ig_vis() 
            plt.xticks(fontsize = fontsize_dis)
            plt.yticks(fontsize = fontsize_dis)

            for sensor in self.sensors_:
                cv = self.vertex_[sensor]
                row = cv.pos_[1] + 0.5
                col = cv.pos_[0] + 0.5
                plt.plot(row,col,"*b",markersize=50)

            # plt.title("Information gain at iteration t = {}".format(self.num_fig), fontsize = fontsize_dis)
            plt.savefig("igimage{}.png".format(self.num_fig))

            print("{} figures have been saved".format(self.num_fig))
            plt.close('all')
            
def main():
    lc = lcm.LCM()
    map = map_vis()
    Entropy_Trend_data = "EntropyTrendData"
    Range_Trend_data = "RangeTrendData"
    Entropy_Trend_Paths_data = "EntropyTrendPathsData"

    subscription = lc.subscribe(Entropy_Trend_data, map.entropy_trend_data_handler)
    subscription = lc.subscribe(Range_Trend_data, map.range_trend_data_handler)
    subscription = lc.subscribe(Entropy_Trend_Paths_data, map.entropy_trend_paths_data_handler)
    for i in range(100000):
        Graph_data = str(i) + "GraphData"
        Path_Collection_data = str(i) + "PathCollectionData"
        Sensors_data = str(i) + "SensorsData"
        Hspots_data = str(i) + "HsptsData"
        subscription = lc.subscribe(Graph_data, map.map_data_handler)
        subscription = lc.subscribe(Path_Collection_data, map.paths_data_handler)
        subscription = lc.subscribe(Hspots_data, map.hspts_data_handler)
        subscription = lc.subscribe(Sensors_data, map.sensors_data_handler)
      

   


    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)


if __name__ == '__main__':
    main()