import numpy as np


class TrajGenRecord(object):

    def __init__(self, save_path, agent_name, num_rollouts):

        self.save_loc = save_path
        self.file_name = agent_name + "_rollout_data"
        self.num_rollouts = num_rollouts
        self.data = None

    def add_rollouts(self, rollouts):

        if self.data is None:
            self.data = np.empty(self.num_rollouts, dtype=object)
            agent_key = np.array(['agent_name', 'pos', 'vel', 'waypoint', 'optimal'],
                                 dtype=object)
            for i, _ in enumerate(rollouts):
                self.data[i] = agent_key

        data = np.empty(self.num_rollouts, dtype=object)
        for i, rollout in enumerate(rollouts):
            rollout_data = np.empty(len(rollout), dtype=object)
            for k, step_data in enumerate(rollout):
                pos = [step_data[6], step_data[7]]
                vel = [step_data[2], step_data[3]]
                waypoint = [step_data[8], step_data[9]]
                optimal = [False]  # To allow tuple concatenation
                rollout_data[k] = pos + vel + waypoint + optimal
            data[i] = rollout_data

        self.data = np.vstack((self.data, data))

    def mark_best_rollout(self, i):
        time_steps = len(self.data[-1, i])
        for step in range(time_steps):
            self.data[-1][i][step][6] = True

    def save_data(self, trial):
        if trial < 10:
            trial_str = '_0' + str(trial)
        else:
            trial_str = '_' + str(trial)
        np.save(self.save_loc + self.file_name + trial_str, self.data)

    def clear_data(self):
        self.data = None
