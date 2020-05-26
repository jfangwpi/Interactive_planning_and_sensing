import json
import matplotlib.pyplot as plt
import numpy as np

num_sensors = [1,2,3,4,5,10,20,30,40,50]
num_tasks = [8,10,12,14,16,18]
num_agents = [2,4,8,10,12,14]

a_ipas_ = {n: [] for n in num_agents}
a_standard_ = {n: [] for n in num_agents}
t_ipas_ = {n: [] for n in num_tasks}
t_standard_ = {n: [] for n in num_tasks}

ipas_actors_ = {n: [] for n in num_agents}
standard_actors_ = {n: [] for n in num_agents}
ipas_tasks_ = {n: [] for n in num_tasks}
standard_tasks_ = {n: [] for n in num_tasks}

# 10-10 map
files_ = ["sim_1sensor","sim_2sensors","sim_3sensors","sim_4sensors","sim_5sensors","sim_10sensors","sim_20sensors","sim_30sensors","sim_40sensors","sim_50sensors"]
path_  = "/home/jfang/Workspace/BIGSIM/10-10-map/"
# 20-20 map
# files_ = ["4sensors","allsenosrs"]
# path_  = "/home/jfang/Workspace/BIGSIM/20-20-map/"

for f in files_:
    f_name = path_ + f + "/ipas-data.json"
    with open(f_name) as json_file:
        data = json_file.read()
        data = data.replace('}{','}\n\n{')
        data = data.split('\n\n')
        ipas_tasks_ = {n: [] for n in num_tasks}
        standard_tasks_ = {n: [] for n in num_tasks}
        ipas_actors_ = {n: [] for n in num_agents}
        standard_actors_ = {n: [] for n in num_agents}
        for id, i in enumerate(data):
            obj = json.loads(i)
            if id % 2 == 0:
                n_a = int(obj["IPAS"]["# of agents"])
                n_t = int(obj["IPAS"]["# of tasks"])
                ipas_tasks_[n_t].append(float(obj["IPAS"]["# of Iteration"]))
                ipas_actors_[n_a].append(float(obj["IPAS"]["# of Iteration"]))
                # ipas_[n_t].append(float(obj["IPAS"]["Entropy Reduction"]))
            # else:
            #     n_t = int(int(obj["Standard IG"]["# of agents"]))
            #     standard_[n_t].append(float(obj["Standard IG"]["# of Iteration"]))
            #     # standard_[n_t].append(float(obj["Standard IG"]["Entropy Reduction"]))

        for item in ipas_actors_.keys():
            ave_ipas_ = sum(ipas_actors_[item])/len(ipas_actors_[item])
            # ave_standard_ = sum(standard_[item])/len(standard_[item])
            a_ipas_[item].append(ave_ipas_)
            # t_standard_[item].append(ave_standard_)

        for item in ipas_tasks_.keys():
            ave_ipas_ = sum(ipas_tasks_[item])/len(ipas_tasks_[item])
            t_ipas_[item].append(ave_ipas_)


# print("Ave iteration for IPAS is {}".format(t_ipas_))
# print("Ave iteration for standard IG is {}".format(t_standard_))
fsize = 2
plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
plt.subplot(2,1,1)
plt.title("Complexity Analysis: # of actors and tasks",fontsize=20)
# plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
plt.plot(num_sensors, a_ipas_[2],'r-',linewidth = fsize,label="# of actors = 2")
plt.plot(num_sensors, a_ipas_[4], 'b-',linewidth = fsize,label="# of actors = 4")
plt.plot(num_sensors, a_ipas_[8], 'y-',linewidth = fsize,label="# of actors = 8")
plt.plot(num_sensors, a_ipas_[10],'g-',linewidth = fsize,label="# of actors = 10")
plt.plot(num_sensors, a_ipas_[12],'m-',linewidth = fsize,label="# of actors = 12")
plt.plot(num_sensors, a_ipas_[14],'k-',linewidth = fsize,label="# of actors = 14")
plt.xlabel("# of sensors",fontsize=20)
# plt.ylabel("Entropy Reduction")
plt.ylabel("# of iterations",fontsize=20)
plt.xticks(fontsize = 20)
plt.yticks(fontsize = 20)
plt.legend(fontsize=20)

plt.subplot(2,1,2)
plt.plot(num_sensors, t_ipas_[8],'r-',linewidth = fsize,label="# of tasks = 8")
plt.plot(num_sensors, t_ipas_[10], 'b-',linewidth = fsize,label="# of tasks = 10")
plt.plot(num_sensors, t_ipas_[12], 'y-',linewidth = fsize,label="# of tasks = 12")
plt.plot(num_sensors, t_ipas_[14],'g-',linewidth = fsize,label="# of tasks = 14")
plt.plot(num_sensors, t_ipas_[16],'m-',linewidth = fsize,label="# of tasks = 16")
plt.plot(num_sensors, t_ipas_[18],'k-',linewidth = fsize,label="# of tasks = 18")
plt.xlabel("# of sensors",fontsize=20)
# plt.ylabel("Entropy Reduction")
plt.ylabel("# of iterations",fontsize=20)
plt.xticks(fontsize = 20)
plt.yticks(fontsize = 20)

# plt.subplot(2,1,2)
# plt.plot(num_sensors, t_standard_[2],'r--',linewidth = fsize,label="# of actors = 2")
# plt.plot(num_sensors, t_standard_[4],'b--',linewidth = fsize,label="# of actors = 4")
# plt.plot(num_sensors, t_standard_[8],'y--',linewidth = fsize,label="# of actors = 8")
# plt.plot(num_sensors, t_standard_[10],'g--',linewidth = fsize,label="# of actors = 10")
# plt.plot(num_sensors, t_standard_[12],'m--',linewidth = fsize,label="# of actors = 12")
# plt.plot(num_sensors, t_standard_[14],'k--',linewidth = fsize,label="# of actors = 14")

# plt.plot(num_sensors, t_standard_,'b-',label="Standard IG")
plt.legend(fontsize=20)


plt.savefig("Complexity_Analysis.png")
plt.show()

