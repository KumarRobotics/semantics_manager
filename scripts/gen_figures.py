#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# dist over time to reach goal from beginning
def plot_goal_reached_times(path):
    d = np.load(path)
    # easier to deal with nans than 0
    d[d == 0] = np.nan
    goal_t = np.nanmin(d[3,:,:], axis=0)
    print(f"{np.count_nonzero(np.isnan(goal_t))} unreached goals")
    # remove nans for plotting
    goal_t = goal_t[~np.isnan(goal_t)]

    fig, ax = plt.subplots()
    ax.violinplot(dataset=[goal_t])

# dist over time to reach goal from when claimed
def plot_nav_times(path):
    d = np.load(path)

# analyze the time robots spend with an active goal vs not
def plot_active_times(path):
    d = np.load(path)

if __name__ == '__main__':
    datapath = '/media/ian/ResearchSSD/Jackal/spomp_sim/test_lawnmower_10robots_0_2023-06-08-16-17-50_goaltimes.npy'
    plot_goal_reached_times(datapath)
    plot_nav_times(datapath)

    plt.show()
