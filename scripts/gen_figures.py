#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import argparse

# dist over time to reach goal from beginning
def plot_goal_reached_times(paths):
    all_goal_t = np.array([])
    for path in paths:
        d = np.load(path)["times"]
        # easier to deal with nans than 0
        d[d == 0] = np.nan
        goal_t = np.nanmin(d[3,:,:], axis=0)
        print(f"{np.count_nonzero(np.isnan(goal_t))} unreached goals")
        # remove nans for plotting
        all_goal_t = np.append(all_goal_t, goal_t[~np.isnan(goal_t)])

    fig, ax = plt.subplots()
    ax.violinplot(dataset=[all_goal_t])

# dist over time to reach goal from when claimed
def plot_nav_times(paths):
    all_diff_t = np.array([])
    for path in paths:
        d = np.load(path)["times"]
        d_supp = np.load(path, allow_pickle=True)["supp"][()]
        d[d == 0] = np.nan
        
        goal_t = np.nanmin(d[3,:,:], axis=0)
        # filter to only worry about goals that were reached
        d = d[:,:,~np.isnan(goal_t)] 
        robot_reached = np.nanargmin(d[3,:,:], axis=0)

        goal_t = d[3,robot_reached,np.arange(d.shape[2])]

        for robot_id in d_supp: 
            for goal_id in d_supp[robot_id]:
                if d_supp[robot_id][goal_id][-1][0] == "claimed":
                    d[1,robot_id,goal_id] = d_supp[robot_id][goal_id][-1][1] 
                elif not np.isnan(d[3,robot_id,goal_id]):
                    print("Something weird is going on here, investigate further")

        claimed_t = d[1,robot_reached,np.arange(d.shape[2])]
        all_diff_t = np.append(all_diff_t, goal_t - claimed_t)

    fig, ax = plt.subplots()
    ax.violinplot(dataset=[all_diff_t])

# analyze the time robots spend with an active goal vs not
def plot_active_times(paths):
    all_robot_t = None
    total_t = 0
    for path in paths:
        d = np.load(path)["times"]
        d_supp = np.load(path, allow_pickle=True)["supp"][()]
        d[d == 0] = np.nan
        end_t = np.nanmax(d)

        robot_t = np.zeros((d.shape[1],))
        for robot_id in range(d.shape[1]):
            robot_d = d[:, robot_id, :]
            unvisited_t = robot_d[2, :] - robot_d[1, :]
            visited_t = robot_d[3, :] - robot_d[1, :]
            if robot_id in d_supp:
                r_supp = d_supp[robot_id]
                for goal_id in r_supp:
                    visited_t[goal_id] = 0
                    last_claimed_t = None
                    for switch in r_supp[goal_id]:
                        if switch[0] == "claimed":
                            last_claimed_t = switch[1]
                        else:
                            visited_t[goal_id] += switch[1] - last_claimed_t

                    if robot_d[3, goal_id] > 0:
                        visited_t[goal_id] += robot_d[3, goal_id] - last_claimed_t

            robot_t[robot_id] = np.nansum(unvisited_t) + np.nansum(visited_t)

        total_t += end_t
        if all_robot_t is None:
            all_robot_t = robot_t
        else:
            all_robot_t = np.vstack((all_robot_t, robot_t))

    fig, ax = plt.subplots()
    ax.bar(range(10), np.sum(all_robot_t, axis=0)/total_t)

# analyze which robots visited goals
def plot_robot_reached(paths):
    all_robot_goal_num = None
    for path in paths:
        d = np.load(path)["times"]

        robot_goal_num = np.zeros((d.shape[1],))
        for robot_id in range(d.shape[1]):
            robot_goal_num[robot_id] = np.count_nonzero(d[3:, robot_id, :])

        if all_robot_goal_num is None:
            all_robot_goal_num = robot_goal_num
        else:
            all_robot_goal_num = np.vstack((all_robot_goal_num, robot_goal_num))

    fig, ax = plt.subplots()
    ax.bar(range(10), np.sum(all_robot_goal_num, axis=0))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate statistics for reaching goals")
    parser.add_argument("data", help="Path to data to process", nargs='+')
    args = parser.parse_args()

    plot_goal_reached_times(args.data)
    plot_nav_times(args.data)
    plot_active_times(args.data)
    plot_robot_reached(args.data)

    plt.show()
