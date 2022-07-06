import matplotlib.pyplot as plt
from matplotlib.pyplot import cm
import numpy as np

ids = [200, 1000, 1800, 2400]

color = cm.rainbow(np.linspace(0, 1, 3))
cl = ['m', 'g', 'r']

plt.figure(figsize=(12,10))
for p in range(4):
    points = np.load('npy/points_'+str(ids[p])+'.npy')
    vor_indices = np.load('npy/vor_indices_'+str(ids[p])+'.npy')
    generators = np.load('npy/generators_'+str(ids[p])+'.npy')
    centroids = np.load('npy/centroids_'+str(ids[p])+'.npy')
    n_generator = len(generators)
    plt.subplot(2,2,p+1)
    for i in range(n_generator):
        plt.plot(points[vor_indices==i,1],points[vor_indices==i,0], '.', markersize=0.1, c=color[i])
    for i in range(n_generator):
        plt.plot(generators[i,1], generators[i,0], 'o', markersize=6, color=cl[i])
        plt.plot(centroids[i,1], centroids[i,0], '*', markersize=10, color=cl[i])
    plt.xlim(10,-10)
    plt.ylim(-10,10)
    plt.title('iteration '+str(ids[p]/20))
plt.show()