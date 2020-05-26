import numpy as np
import math
from matplotlib import pyplot as plt

dimension = [5,10,15,20,25,30,35,40]
optimality = [0.973439169331861,0.960019582690297,0.984830165886203,0.996334288861837,0.997906407628427,
            0.998068903800082,1.0,0.999598367136058]
relative_err = [0.0292571796139113,0.0464127700605231,0.0277762270724065,0.0195579986678243,0.0153087481713879,
            0.0135635196569157,0.0127093696639882,0.00776121210056648]
iterations = [8.05,9.11,10.6,13.97,19.05,29.1,35.1,61.5]
samples = [i+2 for i in iterations]
cases = [100,91,100,98,100,93,92,77]
correct_pert = [i/100 for i in cases]

plt.figure()
plt.subplot(3,1,1)
plt.plot(dimension,optimality,'r-',lw=2)
plt.ylabel('Optimality',fontsize=15)

plt.subplot(3,1,2)
plt.plot(dimension,relative_err,'r-',lw=2)
plt.ylabel("Relative error",fontsize=15)

plt.subplot(3,1,3)
plt.plot(dimension,samples,'r-',lw=2)
plt.ylabel("Number of samples (iterations)",fontsize=15)

# plt.subplot(4,1,4)
# plt.plot(dimension,correct_pert,'r-',lw=2)
# plt.ylabel("Correct percentage",fontsize=15)
plt.show()
