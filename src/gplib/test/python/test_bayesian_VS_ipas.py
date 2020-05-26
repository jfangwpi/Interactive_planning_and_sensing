import json
import matplotlib.pyplot as plt
import numpy as np

bayesian_training_data_ = []
nzig_data_ = []
bayesian_estimated_ig_ = []
ipas_actual_ig_ = []
iteration = []

# open json file
file = "/home/jodie/Workspace/bayesian_opt/IPAS-Bayesian/BayesianResults.json"
with open(file) as json_file:
    data = json_file.read()
    data = data.replace('}{','}\n\n{')
    data = data.split('\n\n')
    for id, i in enumerate(data):
        obj = json.loads(i)
        nzig_data_.append(int(obj["NZIG"]))
        bayesian_training_data_.append(int(obj["Training Data"]))

        ipas_actual_ig_.append(float(obj["IPAS"]))
        bayesian_estimated_ig_.append(float(obj["Bayesian"]))

        iteration.append(int(obj["Iteration"]))

plt.figure()
plt.title("Number of vertices required IG computation",fontsize=20)
plt.plot(iteration,nzig_data_,'b-',LineWidth=3,label="IPAS")
plt.plot(iteration,bayesian_training_data_,'r-',LineWidth=3,label="Bayesian Optimization")
plt.legend(fontsize=20)
# plt.ylabel("Computed IG vertices",fontsize=20)
plt.xlabel("Iteration",fontsize=20)
plt.xlim([0,iteration[-1]])
plt.xticks(fontsize = 20)
plt.yticks(fontsize = 20)

data_percentage = [bayesian_training_data_[i]/nzig_data_[i] for i in range(len(iteration))]
plt.figure()
plt.plot(iteration, data_percentage,'b-',LineWidth=3)
plt.ylabel("Training data/W''",fontsize=20)
plt.xlabel("Iteration",fontsize=20)
plt.xlim([0,iteration[-1]])
plt.xticks(fontsize = 20)
plt.yticks(fontsize = 20)

# plt.figure()
accurancy = [(ipas_actual_ig_[i]-bayesian_estimated_ig_[i])/ipas_actual_ig_[i] for i in range(len(iteration))]
# plt.plot(iteration,accurancy)
relative_error = 1-sum(accurancy)/len(accurancy)
print("The relative error for Bayesian Optimization is {}".format(relative_error))

plt.show()