import json
import matplotlib.pyplot as plt
import numpy as np

ipas_ = []
standard_ = []
t_ipas_ = []
t_standard_ = []

num_sensors = [1,2,3,4,5,10,20,30]

files_ = ["sim_1sensor","sim_2sensors","sim_3sensors","sim_4sensors","sim_5sensors","sim_10sensors","sim_20sensors","sim_30sensors"]
path_  = "/home/jfang/Workspace/BIGSIM/"

for f in files_:
    f_name = path_ + f + "/ipas-data.json"
    with open(f_name) as json_file:
        data = json_file.read()
        data = data.replace('}{','}\n\n{')
        data = data.split('\n\n')
        ipas_ = []
        standard_ = []
        for id, i in enumerate(data):
            obj = json.loads(i)
            if id % 2 == 0:
                ipas_.append(float(obj["IPAS"]["Entropy Reduction"]))
            else:
                standard_.append(float(obj["Standard IG"]["Entropy Reduction"]))
                
        ave_ipas_ = sum(ipas_)/len(ipas_)
        ave_standard_ = sum(standard_)/len(standard_)

        t_ipas_.append(ave_ipas_)
        t_standard_.append(ave_standard_)


print("Ave iteration for IPAS is {}".format(t_ipas_))
print("Ave iteration for standard IG is {}".format(t_standard_))

plt.plot(num_sensors, t_ipas_,'r-',label="IPAS")
plt.plot(num_sensors, t_standard_,'b-',label="Standard IG")
# plt.plot(num_sensors_, en_red_ipas_,'r-',label="IPAS")
# plt.plot(num_sensors_, en_red_standard_,'b-',label="Standard IG")
plt.legend()
plt.xlabel("# of sensors")
plt.ylabel("Map Entropy Reduction")
plt.show()

