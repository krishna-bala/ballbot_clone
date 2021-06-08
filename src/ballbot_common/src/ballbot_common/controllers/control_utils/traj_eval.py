import numpy as np
import rospy
from geometry_msgs.msg import Pose2D


class TrajEval(object):

    def __init__(self, cost_params, num_obs, vpref, dt, rollout_horizon, radius):

        self.cost_params = cost_params
        self.num_obs = num_obs
        self.vpref = vpref
        self.dt = dt
        self.rollout_horizon = rollout_horizon
        self.radius = radius
        self.time_steps = int(rollout_horizon / dt)
        self.init_cost_params()
        self.goal = None

    # noinspection PyAttributeOutsideInit
    def init_cost_params(self):
        self.q = self.cost_params['q']
        self.q_obs = self.q['obs']
        self.q_goal = self.q['goal']
        self.q_wind = self.q['wind']

        self.sigma = self.cost_params['sigma']
        self.sigma_h = self.sigma['h']
        self.sigma_s = self.sigma['s']
        self.sigma_r = self.sigma['r']

        # Normalization factors for Q weights
        self.q_goal_norm = np.square(2 / float(self.rollout_horizon))

        # Empirically found factor of 30
        q_wind_norm = 30 * np.deg2rad(360)
        self.q_wind_norm = np.square(q_wind_norm)

        # Normalized weights
        self.Q_obs = self.q_obs
        self.Q_goal = self.q_goal / self.q_goal_norm
        self.Q_wind = self.q_wind / self.q_wind_norm

    def set_goal(self, goal):
        assert (isinstance(goal, tuple))
        self.goal = goal

    def evaluate_cost(self, ref_traj, traj_id):
        """
        :param ref_traj:
            2D list. [(10 + num_obs * 4), (time_steps + 1)]
            e.g. ref_traj[0][:] -- timestep = 0, all states in traj:
            [0, 0, vx, 0, 0, vy, x, y, gx, gy, obs_x, obs_y, obs_vx, obs_vy]

        :param traj_id: int

        :return total_cost: float
        """
        cost_obs = 0
        cost_goal = 0
        time_steps = int(self.rollout_horizon / self.dt)
        gx, gy = self.goal

        init_dx_to_goal = ref_traj[0][6] - gx
        init_dy_to_goal = ref_traj[0][7] - gy
        init_dist_to_goal = np.linalg.norm((init_dx_to_goal, init_dy_to_goal))

        assert (len(ref_traj[0]) - 10) % 4 == 0
        num_obs = int((len(ref_traj[0]) - 10) / 4)

        winding_hist = np.zeros([time_steps + 1, num_obs])

        # Iterate through every state (time_steps + 1) of ref_traj in rollout horizon
        for k in range(time_steps + 1):
            # Iterate through every obstacle in the current step
            winding_nums = np.zeros([num_obs])
            for obs_id in range(num_obs):
                i = 4 * num_obs - 4 * obs_id
                cost_obs += self.get_cost_obs(ref_traj[k][:], i)
                cost_goal += self.get_cost_goal(ref_traj, k, init_dist_to_goal)
                winding_nums[obs_id] = self.get_winding_num(ref_traj, k, i)
            winding_hist[k] = winding_nums

        cost_winding = self.get_cost_winding(winding_hist)
        total_cost = cost_goal + cost_obs + cost_winding

        # rospy.loginfo("Traj: {}".format(traj_id))
        rospy.loginfo("Cost to goal: {}".format(cost_goal))
        rospy.loginfo("Cost to obs: {}".format(cost_obs))
        # rospy.loginfo("Winding cost: {}".format(cost_winding))
        rospy.loginfo("=====================")
        rospy.loginfo("\n")

        return total_cost

    def get_cost_obs(self, ref_traj, i):
        """
        Args:
            ref_traj: reference trajectory at timestep k
            i: current "other agent"

        Returns:
            float: cost to all obstacles at timestep k
        """
        # Calculate distance to each "other agent"
        dx_to_obs = ref_traj[6] - ref_traj[-i] - (2 * self.radius)
        dy_to_obs = ref_traj[7] - ref_traj[-i + 1] - (2 * self.radius)

        obs_vx, obs_vy = ref_traj[-i + 2], ref_traj[-i + 3]

        # Heading of "other agent"
        theta = np.arctan2(obs_vy, obs_vx)

        # Sigma values used to create 2D gaussian around obstacles for cost penalty
        vel_obs = np.linalg.norm((obs_vx, obs_vy))
        if vel_obs < 0.01:
            sigma_h = 1.0
            sigma_s = 1.0
            sigma_r = 1.0
        else:
            sigma_h = self.sigma_h
            sigma_r = self.sigma_r
            sigma_s = self.sigma_s

        # Alpha calculates whether ego agent is in front or behind "other agent"
        alpha = self.wrap(np.arctan2(dy_to_obs, dx_to_obs) - theta + np.pi / 2.0)
        if alpha <= 0:
            sigma = sigma_r
        else:
            sigma = sigma_h

        # Variables used in cost_obs function based on sigma and theta
        a = np.cos(theta) ** 2 / (2 * sigma ** 2) + np.sin(theta) ** 2 / (2 * sigma_s ** 2)
        b = np.sin(2 * theta) / (4 * sigma ** 2) - np.sin(2 * theta) / (4 * sigma_s ** 2)
        c = np.sin(theta) ** 2 / (2 * sigma ** 2) + np.cos(theta) ** 2 / (2 * sigma_s ** 2)

        # Sum cost across all "other agents"
        cost_obs = np.exp(- ((a * dx_to_obs ** 2) + (2 * b * dx_to_obs * dy_to_obs) +
                             (c * dy_to_obs ** 2))) * self.Q_obs

        return cost_obs

    def get_cost_goal(self, ref_traj, k, init_dist_to_goal):
        # Cost to REAL goal for ego agent (not waypoint)
        dx_to_goal = ref_traj[k][6] - self.goal[0]
        dy_to_goal = ref_traj[k][7] - self.goal[1]
        dist_to_goal = np.linalg.norm((dx_to_goal, dy_to_goal))

        # Straight line distance to goal for "optimal" action at step k:
        str_line = self.vpref * (k * self.dt / self.rollout_horizon)

        # If we overshoot goal with "optimal" action,
        # set str_line to initial dist to goal
        if str_line > init_dist_to_goal:
            str_line = init_dist_to_goal

        optimal_dist_to_goal = init_dist_to_goal - str_line

        if k > 0:
            goal_val = (dist_to_goal - optimal_dist_to_goal) / (2 * float(str_line))
        else:
            goal_val = 0

        cost_goal = goal_val * self.Q_goal * goal_val

        return cost_goal

    @staticmethod
    def get_winding_num(ref_traj, k, i):
        if k > 0:
            dx_to_obs = ref_traj[k][6] - ref_traj[k][-i]
            dy_to_obs = ref_traj[k][7] - ref_traj[k][-i + 1]
            # theta_to_obs = np.arctan2(dy_to_obs, dx_to_obs)
            theta_to_obs = np.arctan2(np.square(dy_to_obs), np.square(dx_to_obs))

            prev_dx_to_obs = ref_traj[k - 1][6] - ref_traj[k - 1][-i]
            prev_dy_to_obs = ref_traj[k - 1][7] - ref_traj[k - 1][-i + 1]
            # prev_theta_to_obs = np.arctan2(prev_dy_to_obs, prev_dx_to_obs)
            prev_theta_to_obs = np.arctan2(np.square(prev_dy_to_obs), np.square(prev_dx_to_obs))

            winding_num = theta_to_obs - prev_theta_to_obs
        else:
            winding_num = 0

        return winding_num

    def get_cost_winding(self, winding_hist):
        winding_val = self.get_winding_val(winding_hist)
        cost_winding = -1 * (winding_val * self.Q_wind * winding_val)
        return cost_winding

    @staticmethod
    def get_winding_val(winding_hist):
        score_per_agent = np.abs(np.sum(winding_hist, axis=0))
        total_score = np.sum(score_per_agent)
        return total_score

    @staticmethod
    def wrap(angle):  # keep angle between [-pi, pi]
        while angle >= np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
