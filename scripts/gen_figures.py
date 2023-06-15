#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import argparse

# dist over time to reach goal from beginning
def plot_goal_reached_times(comp_paths):
    goal_comp = []
    for paths in comp_paths:
        all_goal_t = np.array([])
        for path in paths:
            d = np.load(path)["times"]
            end_t = np.load(path)["end_t"][()]
            # easier to deal with nans than 0
            d[d == 0] = np.nan
            goal_t = np.nanmin(d[3,:,:], axis=0)
            print(f"{np.count_nonzero(np.isnan(goal_t))} unreached goals")
            # replace nans with end_t
            goal_t[np.isnan(goal_t)] = end_t
            all_goal_t = np.append(all_goal_t, goal_t)
        goal_comp.append(all_goal_t)

    fig, ax = plt.subplots()
    ax.violinplot(dataset=goal_comp, showmeans=True)

# dist over time to reach goal from when claimed
def plot_nav_times(comp_paths):
    diff_comp = []
    for paths in comp_paths:
        all_diff_t = np.array([])
        for path in paths:
            d = np.load(path)["times"]
            d_supp = np.load(path, allow_pickle=True)["supp"][()]
            d[d == 0] = np.nan
            
            goal_t = np.nanmin(d[3,:,:], axis=0)
            # filter to only worry about goals that were reached
            d = d[:,:,~np.isnan(goal_t)] 
            map_to_goals = np.arange(goal_t.shape[0])[~np.isnan(goal_t)]
            robot_reached = np.nanargmin(d[3,:,:], axis=0)

            goal_t = d[3,robot_reached, np.arange(d.shape[2])]

            for robot_id in d_supp: 
                for goal_id in d_supp[robot_id]:
                    if goal_id not in map_to_goals:
                        # goal never reached, so we can ignore
                        continue
                    goal_map_id = np.where(map_to_goals == goal_id)[0][0]
                    if d_supp[robot_id][goal_id][-1][0] == "claimed":
                        d[1,robot_id,goal_map_id] = d_supp[robot_id][goal_id][-1][1] 
                    elif not np.isnan(d[3,robot_id,goal_map_id]):
                        print(f"Something weird is going on here, investigate further. path: {path}, robot: {robot_id}, goal: {goal_map_id}")

            claimed_t = d[1,robot_reached,np.arange(d.shape[2])]
            all_diff_t = np.append(all_diff_t, goal_t - claimed_t)
        diff_comp.append(all_diff_t)

    fig, ax = plt.subplots()
    ax.violinplot(dataset=diff_comp, showmeans=True)

# analyze the time robots spend with an active goal vs not
def plot_active_times(comp_paths):
    comps = []
    for dset_ind, paths in enumerate(comp_paths):
        all_robot_unvis_t = None
        all_robot_vis_t = None
        total_t = 0
        for path in paths:
            d = np.load(path)["times"]
            d_supp = np.load(path, allow_pickle=True)["supp"][()]
            d[d == 0] = np.nan

            goal_t = np.nanmin(d[3,:,:], axis=0)
            end_t = np.load(path)["end_t"][()]

            if np.all(~np.isnan(goal_t)):
                # all goals visited, so end time is visit time of last goal
                end_t = np.max(goal_t)

            total_t += end_t
            robot_unvisited_t = np.zeros((d.shape[1],))
            robot_visited_t = np.zeros((d.shape[1],))
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
                                unvisited_t[goal_id] += switch[1] - last_claimed_t

                        if robot_d[3, goal_id] > 0:
                            visited_t[goal_id] += robot_d[3, goal_id] - last_claimed_t
                        elif len(r_supp[goal_id]) > 0 and r_supp[goal_id][-1][0] == "claimed":
                            # in progress at the end
                            unvisited_t[goal_id] += end_t - last_claimed_t

                robot_unvisited_t[robot_id] = np.nansum(unvisited_t)
                robot_visited_t[robot_id] = np.nansum(visited_t)

            if all_robot_vis_t is None:
                all_robot_unvis_t = robot_unvisited_t[None, :]
                all_robot_vis_t = robot_visited_t[None, :]
            else:
                all_robot_unvis_t = np.vstack((all_robot_unvis_t, robot_unvisited_t))
                all_robot_vis_t = np.vstack((all_robot_vis_t, robot_visited_t))

        all_robot_vis_t = np.sum(all_robot_vis_t, axis=0)/total_t
        all_robot_unvis_t = np.sum(all_robot_unvis_t, axis=0)/total_t
        comps.append(all_robot_vis_t)
        comps.append(all_robot_unvis_t + all_robot_vis_t)

        fig, ax = plt.subplots()
        vis_bar = ax.bar(range(all_robot_vis_t.shape[0]), all_robot_vis_t)
        unvis_bar = ax.bar(range(all_robot_vis_t.shape[0]), all_robot_unvis_t, bottom = all_robot_vis_t)
        plt.legend((unvis_bar[0], vis_bar[0]), ("Unvis", "Vis"))
        plt.title(f"Dataset {dset_ind}")

    fig, ax = plt.subplots()
    ax.violinplot(dataset=comps, positions=[i for i in range(len(comps)//2) for _ in range(2)], showmeans=True)

# analyze which robots visited goals
def plot_robot_reached(comp_paths):
    for dset_ind, paths in enumerate(comp_paths):
        all_robot_goal_num = None
        for path in paths:
            d = np.load(path)["times"]

            robot_goal_num = np.zeros((d.shape[1],))
            for robot_id in range(d.shape[1]):
                robot_goal_num[robot_id] = np.count_nonzero(d[3:, robot_id, :])

            if all_robot_goal_num is None:
                all_robot_goal_num = robot_goal_num[None, :]
            else:
                all_robot_goal_num = np.vstack((all_robot_goal_num, robot_goal_num))

        fig, ax = plt.subplots()
        ax.bar(range(all_robot_goal_num.shape[1]), np.sum(all_robot_goal_num, axis=0))
        plt.title(f"Dataset {dset_ind}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate statistics for reaching goals")
    parser.add_argument("-d", "--data", help="Path to data to process", nargs='+', action='append')
    args = parser.parse_args()

    plot_goal_reached_times(args.data)
    plot_nav_times(args.data)
    plot_active_times(args.data)
    plot_robot_reached(args.data)

    plt.show()
