import numpy as np
import matplotlib.pyplot as plt

f1=np.load("C:\\Users\\vpishara\\Downloads\\MRS\\fit_np"+str(1)+".npy")
f2=np.load("C:\\Users\\vpishara\\Downloads\\MRS\\fit_np"+str(2)+".npy")

plt.plot(f1[0,:], f1[1,:],'xr')
plt.plot(f2[0,:], f2[1,:],'xk')
plt.axis([0,100,0,100])
plt.grid(True)
plt.show()