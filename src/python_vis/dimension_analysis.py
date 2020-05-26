import json
import matplotlib.pyplot as plt
import numpy as np
import csv

sensor_density = [0.01,0.02,0.03,0.04,0.05,0.1,0.2,0.3,0.4,0.5]
# map_dim = [10,20,30,40,50]
map_dim = [50]
num_tasks = [10]
num_agents = [4]

t_ipas_ = {n: [] for n in sensor_density}
t_standard_ = {n: [] for n in sensor_density}

ipas_ = {n: [] for n in sensor_density}
standard_ = {n: [] for n in sensor_density}

# # 10-10 map
# files_ = ["sim_1sensor","sim_2sensors","sim_3sensors","sim_4sensors","sim_5sensors","sim_10sensors","sim_20sensors","sim_30sensors", "sim_40sensors", "sim_50sensors"]
# path_  = "/home/jfang/Workspace/BIGSIM/10-10-map/"
# 20-20 map
# files_ = ["4sensors","allsensors"]
files_ = [""]
path_  = "/home/jfang/Workspace/BIGSIM/50-50-map/"

for f in files_:
    # f_name = path_ + f + "/ipas-data.json"
    f_name = path_+"ipas-data.json"
    with open(f_name) as json_file:
        data = json_file.read()
        data = data.replace('}{','}\n\n{')
        data = data.split('\n\n')
        for id, i in enumerate(data):
            obj = json.loads(i)
            
            if (int(obj["IPAS"]["# of agents"]) == 4 and int(obj["IPAS"]["# of tasks"]) == 10):
                n_t = int(obj["IPAS"]["# of col"])
                n_s = int(obj["IPAS"]["# of sensors"])
                p_s = float(n_s)/float(n_t **2)
                # if f == "sim_1sensor": p_s = 0.01
                # elif f == "sim_2sensors": p_s = 0.02
                # elif f == "sim_3sensors": p_s = 0.03
                # elif f == "sim_4sensors": p_s = 0.04
                # elif f == "sim_5sensors": p_s = 0.05
                # elif f == "sim_10sensors": p_s = 0.1
                # elif f == "sim_20sensors": p_s = 0.2
                # elif f == "sim_30sensors": p_s = 0.3
                # elif f == "sim_40sensors": p_s = 0.4
                # elif f == "sim_50sensors": p_s = 0.5

                ipas_[p_s].append(float(obj["IPAS"]["# of Iteration"]))
                # ipas_[n_t].append(float(obj["IPAS"]["Entropy Reduction"]))
            # else:
            #     if (int(obj["Standard IG"]["# of agents"]) == 4 and int(obj["Standard IG"]["# of tasks"]) == 10):
            #         n_t = int(obj["Standard IG"]["# of col"])
            #         n_s = int(obj["Standard IG"]["# of sensors"])
            #         p_s = float(n_s)/float(n_t **2)
            #         # if f == "sim_1sensor": p_s = 0.01
            #         # elif f == "sim_2sensors": p_s = 0.02
            #         # elif f == "sim_3sensors": p_s = 0.03
            #         # elif f == "sim_4sensors": p_s = 0.04
            #         # elif f == "sim_5sensors": p_s = 0.05
            #         # elif f == "sim_10sensors": p_s = 0.1
            #         # elif f == "sim_20sensors": p_s = 0.2
            #         # elif f == "sim_30sensors": p_s = 0.3
            #         # elif f == "sim_40sensors": p_s = 0.4
            #         # elif f == "sim_50sensors": p_s = 0.5
            #         standard_[p_s].append(float(obj["Standard IG"]["# of Iteration"]))
            #         # standard_[n_t].append(float(obj["Standard IG"]["Entropy Reduction"]))

print(ipas_)

for item in ipas_.keys():
    if ipas_[item]:
        ave_ipas_ = sum(ipas_[item])/len(ipas_[item])
    else:
        ave_ipas_ = 0
    # ave_standard_ = sum(standard_[item])/len(standard_[item])
    t_ipas_[item].append(ave_ipas_)
    # t_standard_[item].append(ave_standard_)

it_ipas_ = []
it_standard_ = []
for key in sorted(t_ipas_.keys()):
    it_ipas_.append(t_ipas_[key])

# for key in sorted(t_standard_.keys()):
#     it_standard_.append(t_standard_[key])

with open("50_50_map_4actors_10tasks.cvs", mode="w") as csv_file:
    fieldnames = ["sensor_density", "IPAS_ave_t", "INFO_ave_t"]
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()
    for key in sorted(t_ipas_.keys()):
        writer.writerow({"sensor_density": key, "IPAS_ave_t": t_ipas_[key][0], "INFO_ave_t": 0})

# print("Ave iteration for IPAS is {}".format(t_ipas_))
# print("Ave iteration for standard IG is {}".format(t_standard_))
fsize = 4
plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
print("The size of sensor density is {}".format(len(sensor_density)))
print("The size of t_ipas_ is {}".format(len(t_standard_)))
plt.plot(sensor_density, it_ipas_,'r-',linewidth = fsize,label="Map Dimension = 20*20")
# plt.plot(sensor_density,it_standard_,'b-',linewidth = fsize,label="Map Dimension = 10*10")


# plt.plot(num_sensors, t_standard_,'b-',label="Standard IG")
plt.legend()
plt.title("Complexity Analysis: # of actors and sensors",fontsize=20)
plt.xlabel("# of sensors",fontsize=20)
# plt.ylabel("Entropy Reduction")
plt.ylabel("# of iteration",fontsize=20)
plt.xticks(fontsize = 20)
plt.yticks(fontsize = 20)
# plt.savefig("Complexity_Analysis_10_VS_10_map.png")
plt.show()

