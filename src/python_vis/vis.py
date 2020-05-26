import lcm
import matplotlib.pyplot as plt
import numpy as np
import csv

font_dis = 40
lengend_dis = 35

map_entropy = []
map_entropy_SLAM = []
range_t = []
range_SLAM = []
# with open("Entropy_trend_IPAS.cvs", mode='r') as csv_file:
#     csv_reader = csv.reader(csv_file)
#     next(csv_file)
#     for row in csv_reader:
#         map_entropy.append(row[1])


# with open("Entropy_trend.cvs", mode='r') as csv_file:
#     csv_reader = csv.reader(csv_file)
#     next(csv_file)
#     for row in csv_reader:
#         map_entropy_SLAM.append(row[1])


# plt.figure(num=None, figsize=(25, 15), dpi=80, facecolor='w', edgecolor='k')
# plt.suptitle("Entropy of map",fontsize=font_dis)
# plt.subplot(1,2,1)
# iterations = [i+1 for i in range(len(map_entropy))]
# plt.plot(iterations, map_entropy, 'b-', linewidth=8, label='Proposed method')
# plt.yticks(fontsize = font_dis)
# plt.xticks(fontsize = font_dis)
# plt.xlabel("Num of iteration", fontsize=font_dis)
# # plt.ylabel("Entropy of map", fontsize=font_dis)
# plt.ylim(bottom=0,top=150)
# plt.xlim(left=1, right=iterations[-1])
# plt.legend(fontsize=lengend_dis)
# plt.subplot(1,2,2)
# iterations1 = [i+1 for i in range(len(map_entropy_SLAM))]
# plt.plot(iterations1, map_entropy_SLAM, 'r-', linewidth=8, label='Information-driven')
# plt.yticks(fontsize = font_dis)
# plt.xticks(fontsize = font_dis)
# plt.xlabel("Num of iteration", fontsize=font_dis)
# # plt.ylabel("Entropy of map", fontsize=font_dis)
# plt.ylim(bottom=0,top=150)
# plt.xlim(left=1,right=iterations1[-1])
# plt.legend(fontsize=lengend_dis)

# plt.savefig("entropy_map.png")

range_ = {1:[],2:[],3:[]}
colors = ['black','green','red','black','green']
with open("Range_trend_IPAS.cvs", mode='r') as csv_file:
    csv_reader = csv.reader(csv_file)
    next(csv_file)
    for row in csv_reader:
        range_[1].append(row[1])
        range_[2].append(row[2])
        range_[3].append(row[3])

# with open("Range_trend.cvs", mode='r') as csv_file:
#     csv_reader = csv.reader(csv_file)
#     next(csv_file)
#     for row in csv_reader:
#         range_SLAM.append(row[1])

range_SLAM = np.ones(50)*400


plt.figure(num=None, figsize=(20,12), dpi=80, facecolor='w', edgecolor='k')
plt.suptitle("Dimension of sub-region", fontsize=font_dis)
plt.subplot(1,2,1)
for item in range_.keys():
    iter = [i for i in range(len(range_[item]))]
    plt.plot(iter, range_[item], colors[item], linewidth=8, label="Actor "+str(item))
plt.ylim(top=500,bottom=0)
plt.yticks(np.arange(0, 600, 100))
plt.xlim(left=1)
plt.yticks(fontsize = font_dis)
plt.xticks(fontsize = font_dis)
plt.xlabel("Num of iterations", fontsize=font_dis)
plt.legend(fontsize=lengend_dis)
# plt.ylabel("Dimensiton of sub-region", fontsize=font_dis)
plt.subplot(1,2,2)
iterations1 = [i+1 for i in range(len(range_SLAM))]
plt.plot(iterations1, range_SLAM, 'r-', linewidth=8, label='Information-driven')
plt.ylim(top=500)
plt.xlim(left=1)
plt.yticks(fontsize = font_dis)
plt.xticks(fontsize = font_dis)
plt.ylim(top=500,bottom=0)
plt.yticks(np.arange(0, 600, 100))
plt.xlim(left=1)
plt.xlabel("Num of iterations", fontsize=font_dis)
# plt.ylabel("Dimension of sub-region", fontsize=font_dis)
plt.legend(fontsize=lengend_dis)

plt.savefig("range.png")


paths_entropy = {1:[],2:[],3:[]}
colors = ['black','green','red','black','green']
with open("Entropy_trend_paths_IPAS.cvs", mode='r') as csv_file:
    csv_reader = csv.reader(csv_file)
    next(csv_file)
    for row in csv_reader:
        paths_entropy[1].append(row[1])
        paths_entropy[2].append(row[2])
        paths_entropy[3].append(row[3])
        # paths_entropy[4].append(row[4])


plt.figure(num=None, figsize=(25, 25), dpi=80, facecolor='w', edgecolor='k')
for item in paths_entropy.keys():
    iter = [i for i in range(len(paths_entropy[item]))]
    plt.plot(iter, paths_entropy[item], colors[item], linewidth=8, label="Actor "+str(item))
plt.yticks(fontsize = font_dis)
plt.xticks(fontsize = font_dis)
plt.xlabel("Num of iteration", fontsize=font_dis)
plt.ylabel("Entropy of routes", fontsize=font_dis)
plt.ylim(bottom=0)
plt.xlim(left=1,right=iter[-1])
plt.legend(fontsize=lengend_dis)


# plt.savefig("entropy_paths.png")



plt.show()