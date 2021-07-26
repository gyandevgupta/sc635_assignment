import matplotlib.pyplot as plt
import numpy as np

A = 4
a = 1
b = 2

# Determining the roughness of the structure
n = 64
t = np.linspace(0, 2*np.pi, n+1)

x = A*np.cos(a*t)
y = A*np.sin(b*t)

plt.axis("equal")
plt.grid()
plt.plot(x, y)
plt.show()

