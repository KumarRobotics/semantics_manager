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
    d[d == 0] = np.nan
    
    goal_t = np.nanmin(d[3,:,:], axis=0)
    # filter to only worry about goals that were reached
    d = d[:,:,~np.isnan(goal_t)] 
    robot_reached = np.nanargmin(d[3,:,:], axis=0)

    goal_t = d[3,robot_reached,np.arange(d.shape[2])]
    claimed_t = d[1,robot_reached,np.arange(d.shape[2])]

    diff_t = goal_t - claimed_t

    if not np.all(np.isnan(d[2,robot_reached,np.arange(d.shape[2])])):
        print("WARNING: at least one goal was claimed, released, and claimed again.  Results may not be accurate")

    fig, ax = plt.subplots()
    ax.violinplot(dataset=[diff_t])

# analyze the time robots spend with an active goal vs not
def plot_active_times(path):
    d = np.load(path)
    d[d == 0] = np.nan
    end_t = np.nanmax(d)

    robot_t = np.zeros((d.shape[1],))
    for robot_id in range(d.shape[1]):
        robot_d = d[:, robot_id, :]
        unvisited_t = robot_d[2, :] - robot_d[1, :]
        visited_t = robot_d[3, :] - robot_d[1, :]
        robot_t[robot_id] = np.nansum(unvisited_t) + np.nansum(visited_t)

    fig, ax = plt.subplots()
    ax.bar(range(10), robot_t/end_t)

# analyze which robots visited goals
def plot_robot_reached(path):
    d = np.load(path)

    robot_goal_num = np.zeros((d.shape[1],))
    for robot_id in range(d.shape[1]):
        robot_goal_num[robot_id] = np.count_nonzero(d[3:, robot_id, :])

    fig, ax = plt.subplots()
    ax.bar(range(10), robot_goal_num)

if __name__ == '__main__':
    datapath = '/media/ian/ResearchSSD/Jackal/spomp_sim/test_lawnmower_10robots_0_2023-06-08-16-17-50_goaltimes.npy'
    plot_goal_reached_times(datapath)
    plot_nav_times(datapath)
    plot_active_times(datapath)
    plot_robot_reached(datapath)

    plt.show()
