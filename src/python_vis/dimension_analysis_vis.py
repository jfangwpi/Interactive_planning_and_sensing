import lcm
import matplotlib.pyplot as plt
import numpy as np
import csv

font_dis = 35
lengend_dis = 35

map_dim = [10,20,30,40,50]
ipas_t_ = {n:[] for n in map_dim}
info_t_ = {n:[] for n in map_dim}
sensor_density = [0.01,0.02,0.03,0.04,0.05,0.1,0.2,0.3,0.4,0.5]

files = ["10_10_map_4actors_10tasks","20_20_map_4actors_10tasks","30_30_map_4actors_10tasks","40_40_map_4actors_10tasks","50_50_map_4actors_10tasks"]
for idx,f in enumerate(files):
    f = f + ".cvs"
    with open(f, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_file)
        for row in csv_reader:
            ipas_t_[map_dim[idx]].append(float(row[1]))
            info_t_[map_dim[idx]].append(float(row[2]))

num_sensor = {n: [] for n in map_dim}
for m_d in map_dim:
    for ss_d in sensor_density:
        num_sensor[m_d].append(ss_d * m_d **2)

plt.figure(num=None, figsize=(15,15), dpi=80, facecolor='w', edgecolor='k')
plt.suptitle("IPAS: iterations of convergence", fontsize=font_dis)
# plt.subplot(2,1,1)
# for map_d in map_dim:
colors = ['b>-','r^-','y*-','mo-','g<-']
colors_info = ['b.','rs','y+','m:','gD']
for idx, key in enumerate(sorted(ipas_t_.keys())):
    if key != 50:
        plt.plot(sensor_density, ipas_t_[key], colors[idx], linewidth=4, label="Dimension:{} * {}".format(key,key))
    else:
        print(len(ipas_t_[key]))
        plt.plot(sensor_density[:5], ipas_t_[key][:5], colors[idx], linewidth=4, label="Dimension:{} * {}".format(key,key))
plt.yticks(fontsize = font_dis)
plt.xticks(fontsize = font_dis)
plt.xlabel("Sensor density", fontsize=font_dis)
# plt.ylabel("Dimension of sub-region", fontsize=font_dis)
plt.ylim(bottom=0)
plt.xlim(right=0.2)
plt.legend(fontsize=lengend_dis)

# plt.subplot(2,1,2)
# for idx,key in enumerate(sorted(info_t_.keys())):
    # plt.plot(sensor_density, info_t_[key], colors[idx], linewidth=4, label="Dimension:{} * {}".format(key,key))
# 
plt.yticks(fontsize = font_dis)
plt.xticks(fontsize = font_dis)
plt.xlabel("Sensor density", fontsize=font_dis)
# plt.ylabel("Dimension of sub-region", fontsize=font_dis)
plt.ylim(bottom=0)
plt.legend(fontsize=lengend_dis)
plt.savefig("Dimension_Analysis_IPAS.png")
plt.show()