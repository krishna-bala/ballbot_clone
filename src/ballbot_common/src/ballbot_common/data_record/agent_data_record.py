import numpy as np
import time


class AgentDataRecord(object):

    def __init__(self, save_path):
        self.save_path = save_path
        self.file_name = "exp_data"
        self.data = None

    def add_data(self, agents):
        """

        :param agents: Dictionary of Agent objects, key = name (str) and value = Agent obj
        :type agents: dict(str, Agent)
        """
        if self.data is None:
            self.data = np.empty(len(agents), dtype=object)
            agent_key = np.array(['agent_name', 'pos', 'vel', 'rad', 'vpref', 'heading', 'goal', 'sim_heading',
                                  'controller_name', 'timestamp'],
                                 dtype=object)
            for i, _ in enumerate(agents):
                self.data[i] = agent_key

        data = np.empty(len(agents), dtype=object)

        for i, agent_name in enumerate(agents):
            agent = agents[agent_name]
            pos = agent.get_position()
            vel = agent.get_velocity()
            rad = agent.get_radius()
            vpref = agent.get_vpref()
            heading = agent.get_heading()
            goal = agent.get_goal()
            sim_heading = agent.get_sim_heading()
            controller_name = agent.get_controller_type()
            timestamp = time.time()
            agent_data = np.array([agent_name, pos, vel, rad, vpref, heading, goal, sim_heading, controller_name, timestamp],
                                  dtype=object)
            data[i] = agent_data

        self.data = np.vstack((self.data, data))

    def save_data(self, trial):
        if trial < 10:
            trial_str = '_0' + str(trial)
        else:
            trial_str = '_' + str(trial)
        np.save(self.save_path + self.file_name + trial_str, self.data)

    def clear_data(self):
        self.data = None
