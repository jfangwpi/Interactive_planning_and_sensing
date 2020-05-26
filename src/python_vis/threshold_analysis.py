import json
import matplotlib.pyplot as plt
import numpy as np

IPAS_cost = []
standard_cost = []
opt_cost = []
opt_cost_st = []
opt_cost_ipas_st = []

IPAS_iter = []
standard_iter = []
opt_iter = []

IPAS_entropy_red = []
standard_entropy_red = []
opt_entropy_red = []


threshold = []
relative_cost =[]
with open('/home/jfang/Workspace/BIGSIM/threshold_analysis/ipas-data-threshold-30map.json') as json_file:
    data = json_file.read()
    data = data.replace('}{','}\n\n{')
    data = data.split('\n\n')
    for idx,i in enumerate(data):
        case = "Case"+ str(idx+1)
        obj = json.loads(i)
        if float(obj[case]["Total length"]) >= float(obj[case]["Optimal cost"]):
            IPAS_cost.append(float(obj[case]["Total length"]))
            opt_cost.append(float(obj[case]["Optimal cost"]))
            IPAS_iter.append(float(obj[case]["Iterations"]))
            IPAS_entropy_red.append(float(obj[case]["Entropy reduction"]))

            threshold.append(float(obj[case]["threshold"]))
            relative_cost.append((IPAS_cost[-1] - opt_cost[-1])/opt_cost[-1])

pos_relative = [n for n in relative_cost if n >0]
ave_cost = sum(pos_relative)/len(pos_relative)


# plt.figure()
# plt.plot(threshold,opt_cost,'r',linewidth=2.5,label="IPAS")
# plt.plot(threshold,opt_cost_st,'b',linewidth=2.5,label="Standard IG")
# plt.title("Optimality")
# plt.xlabel("Threshold")
# plt.ylabel("(Act cost - Opt cost)/Opt cost")
# plt.legend()

ftsize = 20
linewd = 3
plt.figure(num=None, figsize=(10, 22), dpi=100, facecolor='w', edgecolor='k')
plt.subplot(3,1,1)
plt.title("Analysis of threshold", fontsize=ftsize)
z_cost = np.polyfit(threshold,relative_cost,3)
p_cost = np.poly1d(z_cost)
plt.plot(threshold,relative_cost,'ro-',linewidth=linewd,label="Relative Error")
plt.plot(threshold,p_cost(threshold),'b--',linewidth=linewd,label="Relative Error")
# plt.xlabel("Threshold",fontsize=ftsize)
plt.ylabel("Relative cost error", fontsize=ftsize)
plt.xticks(fontsize=ftsize)
plt.yticks(fontsize=ftsize)
plt.ylim(bottom=0)
plt.xlim(right=0.06)
# plt.plot(threshold,ave_cost*np.ones(len(threshold)),'b--',linewidth=linewd, label="Average Relative Error")
# plt.legend(fontsize=ftsize)
# plt.savefig("Threshold_VS_RelativeError.png")


#plt.xlim(right=0.06)
#plt.plot(threshold,IPAS_cost,'bo-',linewidth=3)
#plt.plot(threshold,IPAS_entropy_red,'yo-',linewidth=3)

plt.subplot(3,1,3)
z = np.polyfit(threshold,IPAS_iter,3)
p = np.poly1d(z)
# print(z)
# plt.title("IPAS:iterations of convergence", fontsize=ftsize)
plt.plot(threshold,IPAS_iter,'ro-',linewidth=linewd,label="Iterations of convergence")
plt.plot(threshold,p(threshold),'b--',linewidth=linewd, label="Polynomial fit")
plt.ylabel("# of iterations",fontsize=ftsize)
plt.xlabel("Threshold",fontsize=ftsize)
plt.xticks(fontsize=ftsize)
plt.yticks(fontsize=ftsize)
plt.xlim(right=0.06)
# plt.legend(fontsize=ftsize)
# plt.savefig("Threshold_VS_Iterations.png")

plt.subplot(3,1,2)
z_er = np.polyfit(threshold,IPAS_entropy_red,3)
p_er = np.poly1d(z_er)
plt.plot(threshold,IPAS_entropy_red,'ro-',linewidth=linewd)
plt.plot(threshold,p_er(threshold),'b--',linewidth=linewd)
plt.ylabel("Entropy reduction", fontsize=ftsize)
plt.xticks(fontsize=ftsize)
plt.yticks(fontsize=ftsize)
plt.xlim(right=0.06)
plt.legend(fontsize=ftsize)
plt.savefig("Analysis_threshold.png")

# plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
# plt.plot(threshold,IPAS_iter,'r',linewidth=2.5,label="IPAS")
# plt.plot(threshold,standard_iter,'b',linewidth=2.5,label="Information-driven")
# plt.xlabel("Threshold")
# plt.ylabel("# of Iterations")
# plt.legend()


# plt.figure()
# plt.plot(threshold,IPAS_entropy_red,'r',linewidth=2.5,label="IPAS")
# plt.plot(threshold,standard_entropy_red,'b',linewidth=2.5, label="Standard IG")
# plt.xlabel("Threshold")
# plt.ylabel("Entropy Reduction")
# plt.legend()

plt.show()