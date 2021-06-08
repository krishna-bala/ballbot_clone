#!/usr/bin/env python
import numpy as np
import rospy

from functools import wraps
from time import time
import logging

from geometry_msgs.msg import Twist

from ballbot_common.controllers.base_controller import BaseController
from ballbot_common.utils.action import ActionXY
# from ballbot_common.data_record.traj_gen_record import TrajGenRecord


class MPC(BaseController):

    def __init__(self):
        super(MPC, self).__init__()

        self.traj_gen = None
        self.traj_eval = None
        self.ego_state = None
        self.other_states = None
        self.sim_heading = None
        self.cost_params = None
        self.traj_gen_record = None

        # self.plotter = Plotter(num_obs)

    def __str__(self):
        return 'MPC with {} rollouts'.format(self.traj_gen)

    def set_traj_gen(self, traj_gen):
        self.traj_gen = traj_gen

    def set_traj_gen_record(self, traj_gen_recorder):
        self.traj_gen_record = traj_gen_recorder

    def set_traj_eval(self, traj_eval):
        self.traj_eval = traj_eval

    def set_cost_params(self, cost_params):
        self.cost_params = cost_params

    # noinspection PyAttributeOutsideInit
    def get_action(self, state, frame):

        assert frame in ['body', 'global']
        # Set ego state and state of other agents
        self.ego_state = state.self_state
        self.other_states = state.other_states

        # Initialize sim heading for MPC if first iteration
        if self.sim_heading is None:
            self.sim_heading = self.ego_state.get_heading()
        else:
            # Set the joint state sim_heading for rollouts
            state.self_state.set_sim_heading(self.sim_heading)

        if self.at_goal():
            return ActionXY(0, 0)

        # Needs updated sim_heading for rollouts
        trajectories = self.traj_gen.generate_rollouts(state)
        self.traj_gen_record.add_rollouts(trajectories)
        true_goal = self.ego_state.get_goal()
        self.traj_eval.set_goal(true_goal)
        best_traj, id = self.evaluate_trajectories(trajectories)
        self.traj_gen_record.mark_best_rollout(id)
        opt_vx = best_traj[1][2]
        opt_vy = best_traj[1][5]

        global_action_xy = ActionXY(opt_vx, opt_vy)

        if frame == 'body':
            # Convert to body frame ActionXY
            action_xy = self.action_global_xy_to_body_xy(global_action_xy)
            print
        else:
            # Keep in global frame ActionXY
            action_xy = global_action_xy

        self.sim_heading = np.arctan2(opt_vy, opt_vx)

        return action_xy

    def action_global_xy_to_body_xy(self, global_action_xy):
        heading = self.ego_state.get_heading()
        body_vx = np.cos(heading) * global_action_xy.vx + np.sin(heading) * global_action_xy.vy
        body_vy = -np.sin(heading) * global_action_xy.vx + np.cos(heading) * global_action_xy.vy

        return ActionXY(body_vx, body_vy)

    def evaluate_trajectories(self, trajectories):
        best_cost = float('inf')
        best_traj = None
        id = None
        for i, traj in enumerate(trajectories):
            traj_cost = self.traj_eval.evaluate_cost(traj, traj_id=i)
            if traj_cost < best_cost:
                best_cost = traj_cost
                best_traj = traj
                id = i

        assert best_traj, id
        return best_traj, id

    # def plot_trajectories(self):
    #     self.plotter.plot_agents(self.state, self.other_agents)
    #     self.plotter.plot_goal(self.goal)
    #     for traj in self.trajectories:
    #         self.plotter.plot_traj(traj)
    #     self.plotter.plot_traj(self.best_traj, best=True)
    #     # self.plotter.plot_other_agent_traj(self.best_traj)

    @staticmethod
    def wrap(angle):
        while angle >= np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
