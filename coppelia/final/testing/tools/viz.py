import numpy as np
import matplotlib.pyplot as plt
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
def plot_clusters(db, data):
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    # n_noise_ = list(labels).count(-1)

    print('Estimated number of clusters: %d' % n_clusters_)

    # #############################################################################
    # Plot result
    cmap = plt.cm.get_cmap("Spectral")
    # Black removed and is used for noise instead.
    unique_labels = set(labels)
    colors = [cmap(each)
            for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = (labels == k)

        xy = data[class_member_mask & core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                markeredgecolor='k', markersize=14)

        xy = data[class_member_mask & ~core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                markeredgecolor='k', markersize=6)

    plt.title('Estimated number of clusters: %d' % n_clusters_)
    plt.show(block=True)

def plot_2d(data):
    fig, ax = plt.subplots()
    plt.plot(data[:,0], data[:,1], 'ko')
    plt.show(block=True)

def plot_grid(grid, path=None, r_c=None, goal=None):
    if path is not None:
        for p in path:
            grid[p] = 2
    fig,ax = plt.subplots()
    plt.imshow(grid.T)
    plt.gca().invert_yaxis()
    if r_c is not None:
        plt.plot(r_c[0], r_c[1], 'c^')
    if goal is not None:
        plt.plot(goal[0], goal[1], marker='+', color='red', ms=20)
        plt.plot(goal[0], goal[1], marker='o', mec='red', mfc='none', ms=15)
    plt.show(block=True)

def plot_3d(data):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(data[:,0], data[:,1], data[:,2])
    plt.show(block=True)
