import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import LogNorm
import lcm
import random

grid_size = [5,10,15,20,25,30,35,40]
L2_norm = [2.27221688771807,2.69322298141664,6.49699962104022,6.41635853844009,9.31726144757102,11.0048857926168,12.312276876543,8.60969863385202]
ave_L2_norm = [0.0908886755087229,0.0269322298141664,0.0288755538712898,0.0160408963461002,0.0149076183161136,0.0122276508806853,0.0100508382665657,0.00538106164615751]

# Complexity = [1,2,3,4,5]
# L2_norm = [0.113124080484095,0.113124080484095,9.149436020477,425.070040145283,1313.02365198776]
# ave_L2_norm = [0.000282810201210237,0.0078271564393379,0.0228735900511925,1.0626751003632,3.28255912996941]

plt.figure()
plt.subplot(1,2,1)
plt.plot(grid_size,L2_norm,'b*-',linewidth=3)
plt.xticks(np.arange(grid_size[0],grid_size[-1]+1,5))
plt.ylabel("Total L2 norm",fontsize=20)
plt.xlabel("Dimension",fontsize=20)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.subplot(1,2,2)
plt.plot(grid_size,ave_L2_norm,'r*-',linewidth=3)
plt.ylabel("Ave L2 norm",fontsize=20)
plt.xlabel("Dimension of map",fontsize=20)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.xticks(np.arange(grid_size[0],grid_size[-1]+1,5))
plt.show()