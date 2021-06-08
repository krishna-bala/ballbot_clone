from matplotlib import pyplot as plt
from datetime import datetime
from datetime import datetime
import os


class TrajGenRecord(object):

    def __init__(self):
        now = datetime.now()
        today = datetime.today()
        self.today = today.strftime("%Y-%m-%d/")
        self.time = now.strftime("%H.%M.%S")
        self.name = self.time + "_rollouts"
        self.save_dir = None


class Plotter(object):

    def __init__(self, num_obs):
        now = datetime.now()
        today = datetime.today()
        self.today = today.strftime("%Y-%m-%d/")
        self.time = now.strftime("%H.%M.%S")
        self.name = self.time + "_rollouts"
        self.save_dir = None
        self.num_obs = num_obs
        plt.figure()

    @staticmethod
    def plot_agents(ego_agent, other_agents):
        for i in range(len(other_agents.name)):
            plt.scatter(other_agents.pose[i].position.x, other_agents.pose[i].position.y, color='m', marker='o',
                        zorder=3)
        plt.scatter(ego_agent.pose.position.x, ego_agent.pose.position.y, color='b', marker='o', zorder=3)

    def plot_goal(self, goal):
        plt.scatter(goal.x, goal.y, color='r', marker='x', zorder=2)

    def plot_trajs(self, trajs):
        for traj in trajs:
            self.plot_traj(traj)

    @staticmethod
    def plot_traj(traj, best=False):
        px = []
        py = []
        for step in traj:
            px.append(step[6])
            py.append(step[7])
        if best:
            color = 'g'
            zorder = 2
        else:
            color = 'y'
            zorder = 1
        plt.plot(px, py, color=color, zorder=zorder)

    def plot_other_agent_traj(self, traj):
        for i in range(self.num_obs):
            px = []
            py = []
            x_pos = 10 + 4 * (i)
            y_pos = 11 + 4 * (i)
            for step in traj:
                px.append(step[x_pos])
                py.append(step[y_pos])
            color = 'r'
            zorder = 1
            plt.plot(px, py, color=color, zorder=zorder)

    def set_pack_dir(self, pack_dir):
        self.save_dir = pack_dir + "/../../log/plots/sim/rollouts/" + self.today
        if not os.path.exists(self.save_dir):
            os.mkdir(self.save_dir)

    def save(self, fig=None):
        try:
            assert (self.save_dir is not None)
        except:
            raise AssertionError
        plt.savefig(self.save_dir + self.name + ".png")
