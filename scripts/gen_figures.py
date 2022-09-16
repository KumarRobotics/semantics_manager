#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import argparse

# dist over time to reach goal from beginning
def plot_goal_reached_times(comp_paths):
    goal_comp = []
    diff_num_unreached = []
    for paths in comp_paths:
        all_goal_t = np.array([])
        total_num_unreached = 0
        for path in paths:
            d = np.load(path)["times"]
            end_t = np.load(path)["end_t"][()]
            # easier to deal with nans than 0
            d[d == 0] = np.nan
            goal_t = np.nanmin(d[3,:,:], axis=0)
            num_unreached = np.count_nonzero(np.isnan(goal_t))
            total_num_unreached += num_unreached
            print(f"{num_unreached} unreached goals")
            print(f"{np.count_nonzero(~np.isnan(goal_t))} reached goals")
            # replace nans with end_t
            goal_t[np.isnan(goal_t)] = end_t
            all_goal_t = np.append(all_goal_t, goal_t)

        goal_comp.append(all_goal_t/60.)
        diff_num_unreached.append(total_num_unreached)

    fig, ax = plt.subplots()
    ax.set_title("Distribution over when goals visited", fontsize=15)
    ax.set_ylabel("Time goal visited (minutes from experiment start)", fontsize=12)
    ax.violinplot(dataset=goal_comp, showmeans=True)
    ax.set_xticks([])
    ax.tick_params(axis='y', labelsize=12)
    for ind, nu in enumerate(diff_num_unreached):
        ax.text(ind+1, np.max(goal_comp[ind]+0.2), f"{nu} not visited", horizontalalignment='center', fontsize=12)

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
        diff_comp.append(all_diff_t/60.)

    fig, ax = plt.subplots()
    ax.violinplot(dataset=diff_comp, showmeans=True)
    ax.set_title("Distribution over times to visit goals", fontsize=15)
    ax.set_ylabel("Duration of time to reach goal\n(minutes from goal being claimed)", fontsize=12)
    ax.set_xticks([])
    ax.tick_params(axis='y', labelsize=12)

# analyze the time robots spend with an active goal vs not
def plot_active_times(comp_paths):
    comps = []
    fig, ax = plt.subplots()
    for dset_ind, paths in enumerate(comp_paths):
        all_robot_unvis_t = None
        all_robot_vis_t = None
        end_times = np.zeros([len(paths)])
        for p_ind, path in enumerate(paths):
            d = np.load(path)["times"]
            d_supp = np.load(path, allow_pickle=True)["supp"][()]
            d[d == 0] = np.nan

            goal_t = np.nanmin(d[3,:,:], axis=0)
            end_t = np.load(path)["end_t"][()]

            if np.all(~np.isnan(goal_t)):
                # all goals visited, so end time is visit time of last goal
                end_t = np.max(goal_t)

            # ignore all times after end_t
            d = np.minimum(d, end_t)

            end_times[p_ind] = end_t
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
                                last_claimed_t = min(switch[1], end_t)
                            else:
                                unvisited_t[goal_id] += min(switch[1], end_t) - last_claimed_t

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

        comps.append((all_robot_vis_t/end_times[:,None]).flatten())
        comps.append(((all_robot_unvis_t + all_robot_vis_t)/end_times[:,None]).flatten())

        all_robot_vis_t = np.sum(all_robot_vis_t, axis=0)/np.sum(end_times)
        all_robot_unvis_t = np.sum(all_robot_unvis_t, axis=0)/np.sum(end_times)

        bar_width = 0.8/(len(comp_paths)+0.2)
        x_loc = np.arange(all_robot_vis_t.shape[0]) + dset_ind/(len(comp_paths)+1)
        vis_bar = ax.bar(x_loc, all_robot_vis_t, color='blue', width=bar_width, align='edge')
        unvis_bar = ax.bar(x_loc, all_robot_unvis_t, bottom = all_robot_vis_t, color='orange', width=bar_width, align='edge')
        plt.legend((unvis_bar[0], vis_bar[0]), ("Not Visited", "Visited"), fontsize=12)
        plt.ylabel("Fraction of time spent en route to goal", fontsize=12)
        plt.xticks(np.arange(all_robot_vis_t.shape[0])+0.4, [f"Robot {i}" for i in range(all_robot_vis_t.shape[0])], rotation='vertical', fontsize=12)
        ax.set_title("Fraction of time spent en route to goal for each robot", fontsize=15)
        ax.tick_params(axis='x', which='both', length=0)
        ax.tick_params(axis='y', labelsize=12)

    fig, ax = plt.subplots()
    ax.set_title("Distribution over fraction of time spent en route to goal", fontsize=15)
    ax.set_ylabel("Fraction of time spent en route to goal", fontsize=12)
    violins = ax.violinplot(dataset=comps, positions=[i for i in range(len(comps)//2) for _ in range(2)], showmeans=True)
    ax.set_xticks([])
    ax.set_ylim([0,1])
    ax.tick_params(axis='y', labelsize=12)

    colors = []
    for v_id, violin in enumerate(violins['bodies']):
        if v_id % 2 != 0:
            colors.append('blue')
            violin.set_facecolor('blue')
            violin.set_edgecolor('blue')
        else:
            colors.append('green')
            violin.set_facecolor('green')
            violin.set_edgecolor('green')

    for key in ('cbars','cmins','cmaxes','cmeans'):
        violins[key].set_color(colors)

    visited_patch = mpatches.Patch(color='green', label='To goal visited')
    all_patch = mpatches.Patch(color='blue', label='To goal')
    ax.legend(handles=[visited_patch, all_patch], fontsize=12)

# analyze which robots visited goals
def plot_robot_reached(comp_paths):
    fig, ax = plt.subplots()
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

        bar_width = 0.8/(len(comp_paths)+0.2)
        x_loc = np.arange(d.shape[1]) + dset_ind/(len(comp_paths)+1)

        ax.bar(x_loc, np.sum(all_robot_goal_num, axis=0), width=bar_width, align='edge')

    plt.xticks(np.arange(d.shape[1])+0.4, [f"Robot {i}" for i in range(d.shape[1])], rotation='vertical', fontsize=12)
    ax.tick_params(axis='x', which='both', length=0)
    ax.tick_params(axis='y', labelsize=12)
    plt.title("Number of goals visited by each robot", fontsize=15)
    plt.legend([f"Dataset {i}" for i in range(len(comp_paths))], fontsize=12)
    plt.ylabel("Number of goals visited", fontsize=12)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate statistics for reaching goals")
    parser.add_argument("-d", "--data", help="Path to data to process", nargs='+', action='append')
    args = parser.parse_args()

    plot_goal_reached_times(args.data)
    plot_nav_times(args.data)
    plot_active_times(args.data)
    plot_robot_reached(args.data)

    plt.show()
