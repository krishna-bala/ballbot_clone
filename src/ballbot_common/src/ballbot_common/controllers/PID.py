#!/usr/bin/env python
import numpy as np

from ballbot_common.controllers.base_controller import BaseController
from ballbot_common.utils.action import ActionRot, ActionXY


class PID(BaseController):

    def __init__(self):

        super(PID, self).__init__()
        self.goal_thresh = 0.2
        self.kp = 1.0
        self.ki = 0.01
        self.kd = 0.01

        self.cum_err = np.array([0, 0])
        self.prev_err = None

        self.ego_state = None
        self.other_states = None
        self.goal = None

    def __str__(self):
        return 'PID'

    def reset_memory(self):
        self.cum_err = np.array([0, 0])
        self.prev_err = None

    def get_action(self, state, frame):

        assert frame in ['body', 'global']

        self.ego_state = state.self_state
        self.other_states = state.other_states

        if self.at_goal():
            return ActionXY(0, 0)
        else:
            # Get global frame ActionXY
            global_action_xy = self.get_pid_action()

            if frame == 'body':
                # Convert to body frame ActionXY
                action_xy = self.action_global_xy_to_body_xy(global_action_xy)
            else:
                action_xy = global_action_xy

            return action_xy

    def action_global_xy_to_body_xy(self, global_action_xy):
        heading = self.ego_state.get_heading()
        body_vx = np.cos(heading) * global_action_xy.vx + np.sin(heading) * global_action_xy.vy
        body_vy = -np.sin(heading) * global_action_xy.vx + np.cos(heading) * global_action_xy.vy

        return ActionXY(body_vx, body_vy)

    def get_pid_action(self):

        px, py = self.ego_state.get_position()
        gx, gy = self.ego_state.get_goal()
        if self.goal != (gx, gy):
            self.reset_memory()
            self.goal = (gx, gy)
        vpref = self.ego_state.get_vpref()

        curr_err = [gx - px, gy - py]

        self.cum_err[0] = self.cum_err[0] + curr_err[0]
        self.cum_err[1] = self.cum_err[1] + curr_err[1]

        if self.prev_err is not None:
            delta_err = [(curr_err[0] - self.prev_err[0]) / self.dt,
                         (curr_err[1] - self.prev_err[1]) / self.dt]
            global_vx = self.kp * curr_err[0] + self.ki * self.cum_err[0] + self.kd * delta_err[0]
            global_vy = self.kp * curr_err[1] + self.ki * self.cum_err[1] + self.kd * delta_err[1]
        else:
            global_vx = self.kp * curr_err[0] + self.ki * self.cum_err[0]
            global_vy = self.kp * curr_err[1] + self.ki * self.cum_err[1]

        self.prev_err = np.array(curr_err)

        v = np.linalg.norm((global_vx, global_vy))
        if v > vpref:
            global_vx *= vpref / v
            global_vy *= vpref / v

        return ActionXY(global_vx, global_vy)

    def at_goal(self):
        pos = self.ego_state.get_position()
        goal = self.ego_state.get_goal()
        radius = self.ego_state.get_radius()
        dist_to_goal = (pos[0] - goal[0], pos[1] - goal[1])
        if np.linalg.norm(dist_to_goal) - radius < self.goal_thresh:
            return True
        return False
