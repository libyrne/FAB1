#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

homedir = str(Path.home())

# data path
datadir = homedir + "/odrive_ws/src/odrive_controller/data/"

# Load values into arrays?
y0 = np.load(datadir + "VelValsAxis0.npy")
y1 = np.load(datadir + "VelValsAxis1.0.npy")
# y8 = np.load(datadir + "VelEst8.0.npy")
# y16 = np.load(datadir + "VelEst16.0.npy")
# y32 = np.load(datadir + "VelEst32.0.npy")

print(y2.shape)

# y_vals = [2, 4, 8, 16, 32]
# data = [y2, y4, y8, y16, y32]

# Plot arrays in scatter plots
for i, d in enumerate(data):
    m0 = np.mean(d[0, :])
    # m1 = np.mean(d[1, :])
    s0 = np.std(d[0, :])
    s1 = np.std(d[1, :])
    cid0 = m0 - 1.96*s0
    ciu0 = m0 + 1.96*s0
    cid1 = m1 - 1.96*s1
    ciu1 = m1 + 1.96*s1

    plt.figure(2*i)
    plt.scatter(range(d.shape[1]), d[0, :], label="axis0 estimate", color="#1D2951")
    plt.plot([0, d.shape[1]], [m0, m0], label="axis0 mean: %.3f" % m0, color="#B19CD9", linewidth=5)
    plt.plot([0, d.shape[1]], [cid0, cid0], label="axis0 lower ci: %.3f" % cid0, color="#6699CC", linewidth=5)
    plt.plot([0, d.shape[1]], [ciu0, ciu0], label="axis0 upper ci: %.3f" % ciu0, color="#6699CC", linewidth=5)
    plt.legend()
    plt.title("axis 0 target velocity: %d" % y_vals[i])
    plt.ylabel("velocity")

    plt.figure(2*i+1)
    plt.scatter(range(d.shape[1]), d[1, :], label="axis1 estimate", color="#1D2951")
    plt.plot([0, d.shape[1]], [m1, m1], label="axis1 mean: %.3f" % m1, color="#B19CD9", linewidth=5)
    plt.plot([0, d.shape[1]], [cid1, cid1], label="axis1 lower ci: %.3f" % cid1, color="#6699CC", linewidth=5)
    plt.plot([0, d.shape[1]], [ciu1, ciu1], label="axis1 upper ci: %.3f" % ciu1, color="#6699CC", linewidth=5)
    plt.legend()
    plt.title("axis 1 target velocity: %d" % y_vals[i])
    plt.ylabel("velocity")
    

plt.show()