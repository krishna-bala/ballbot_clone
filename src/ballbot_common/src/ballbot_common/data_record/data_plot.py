from matplotlib import pyplot as plt
import numpy as np


class DataPlot(object):

    def __init__(self, save_path, agent_data, rollout_data):

        self.save_path = save_path
        self.file_name = "traj_plot.png"
        self.agent_data = agent_data
        self.rollout_data = rollout_data
        plt.figure(dpi=1200)

    def plot_data(self):
        num_agents = len(self.agent_data[0])
        data_points = len(self.agent_data)
        data_pos = np.empty((num_agents, 4), dtype=object)
        data_pos[:, 0] = 'agent_name'
        data_pos[:, 1] = 'px'
        data_pos[:, 2] = 'py'
        data_pos[:, 3] = 'color'
        data_temp = np.empty((num_agents, 4), dtype=object)
        for k in range(1, data_points):
            for agent in range(num_agents):
                agent_name = self.agent_data[k][agent][0]
                agent_pos = self.agent_data[k][agent][1]
                # agent_vel = self.agent_data[k][agent][2]
                if agent_name != "ballbot":
                    agent_color = 'm'
                else:
                    agent_color = 'b'
                    ballbot_goal = self.agent_data[k][agent][6]
                    goal_color = 'r'
                    plt.scatter(ballbot_goal[0], ballbot_goal[1], color=goal_color, marker='x', s=100)

                data_temp[agent, 0] = agent_name
                data_temp[agent, 1] = agent_pos[0]
                data_temp[agent, 2] = agent_pos[1]
                data_temp[agent, 3] = agent_color
            data_pos = np.vstack((data_pos, data_temp))

        for agent in range(num_agents):
            start = num_agents
            agent_name_data = data_pos[(start+agent)::num_agents][:, 0]
            agent_x_data = data_pos[(start+agent)::num_agents][:, 1]
            agent_y_data = data_pos[(start+agent)::num_agents][:, 2]
            agent_color_data = data_pos[(start+agent)::num_agents][:, 3]

            plt.plot(agent_x_data, agent_y_data, color=agent_color_data[0], zorder=4, linewidth=1.0)



            # plt.scatter(agent_goal[0], agent_goal[1], color=goal_color, marker='X', zorder=4)
            # plt.scatter(agent_pos[0], agent_pos[1], color=agent_color, marker='o', zorder=4)

    def plot_rollouts(self):
        data_points = len(self.rollout_data)
        num_rollouts = len(self.rollout_data[0])
        rollout_length = len(self.rollout_data[1][0])
        # Only plot every 10th rollout for easier viewing; skip first entry
        for k in range(1, data_points, 10):
            for rollout in range(num_rollouts):
                # Don't plot first step as it is the same data_record point as agent data_record.
                rollout_x = []
                rollout_y = []

                # Plot the initial position of rollouts at this time step
                start_data = self.rollout_data[k][0][0]
                agent_px = start_data[0]
                agent_py = start_data[1]
                plt.scatter(agent_px, agent_py, color='c', marker='o', s=5, zorder=5)

                for rollout_step in range(rollout_length):
                    step_data = self.rollout_data[k][rollout][rollout_step]
                    rollout_x.append(step_data[0])
                    rollout_y.append(step_data[1])
                    # rollout_waypoint = step_data[4], step_data[5]
                    optimal_rollout = step_data[6]
                    if optimal_rollout:
                        rollout_color = 'g'
                        order = 3
                    else:
                        rollout_color = 'y'
                        order = 2
                    # waypoint_color = 'c'
                plt.plot(rollout_x, rollout_y, color=rollout_color, zorder=order, linewidth=0.2)

    def save_plot(self):
        self.plot_data()
        self.plot_rollouts()
        plt.savefig(self.save_path + self.file_name)
