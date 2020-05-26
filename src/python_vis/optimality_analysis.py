import matplotlib.pyplot as plt
import numpy as np

opt_length = 65
# opt_length = 108
penalty_ = [1,5,10,20,25,30,35,40,45,50,55,60,70,80,100,150,200]
length_ = [71,95,79,73,73,73,73,73,73,79,79,79,79,79,79,79,79]
opt_length_ = [71,95,79,71,71,71,71,71,71,71,71,71,71,71,71,71,71]
print(len(opt_length_))
print(len(length_))
# length_ = [131,144,137,147,144,146,150,143,146,146,146]
entropy_red_ = [0.556041,0.444954,0.43817,0.393076,0.407876,0.395551,0.395551,0.395551,0.395551,0.395551,0.359051,0.359051,0.359051,0.359051,0.359051,0.359051,0.359051]
# entropy_red_ = [0.480616,0.403132,0.290271,0.334034,0.297655,0.278799,0.303404,0.290214,0.289763,0.283139,0.283139]
opt_analysis_ = [float((n-opt_length_[idx]))/opt_length_[idx] for idx,n in enumerate(length_)]

plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
plt.subplot(2,1,1)
plt.title("IPAS: relative paths costs error",fontsize=20)
plt.plot(penalty_,opt_analysis_,'bo-',linewidth=3)
# plt.xlabel("Penalty",fontsize=20)

plt.xticks(fontsize = 20)
plt.yticks(fontsize = 20)
plt.subplot(2,1,2)
plt.title("IPAS: entropy reduction",fontsize=20)
plt.plot(penalty_,entropy_red_,'ro-',linewidth=3)
plt.xlabel("Penalty",fontsize=20)
# plt.ylabel("Entropy Reduction",fontsize=20)
plt.xticks(fontsize = 20)
plt.yticks(fontsize = 20)
plt.savefig("Penalty_VS_RelativeError_VS_Entropy_Reduction.png")
plt.show()