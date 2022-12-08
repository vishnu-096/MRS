import numpy as np
import math
import matplotlib.pyplot as plt

N = 10
t = np.linspace(0, 2*np.pi, N)
x = 5*np.cos(t) + 0.2*np.random.normal(size=N) + 1
y = 4*np.sin(t+0.5) + 0.2*np.random.normal(size=N)
plt.plot(x, y, '.')     # given points

xmean, ymean = x.mean(), y.mean()
x -= xmean
y -= ymean
U, S, V = np.linalg.svd(np.stack((x, y)))

tt = np.linspace(0, 2*np.pi, 1000)
circle = np.stack((np.cos(tt), np.sin(tt)))    # unit circle
transform = np.sqrt(2/N) * U.dot(np.diag(S))   # transformation matrix
fit = transform.dot(circle) + np.array([[xmean], [ymean]])
plt.plot(fit[0, :], fit[1, :], 'r')
# print(np.sqrt(2/N) * U.dot(np.diag(S)))
# print(S)
# print(xmean,ymean)
plt.plot(xmean,ymean,'xk')

a=np.sqrt(2/N)*S[0]
b=np.sqrt(2/N)*S[1]
t=np.linspace(0,2*np.pi,100)
x=a*np.cos(t)
y=b*np.sin(t)
plt.plot(x,y,'b')
plt.show()