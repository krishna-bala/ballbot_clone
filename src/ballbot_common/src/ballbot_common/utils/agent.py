import numpy as np
import abc
import logging

from ballbot_common.controllers.base_controller import BaseController
from ballbot_common.utils.state import ObservableState, ROSState, FullState, JointState
from ballbot_common.utils.action import ActionXY, ActionRot


class Agent(FullState):

    def __init__(self, px, py, vx, vy, radius, vpref, roll, pitch, yaw, gx, gy, sim_heading, dt, action_frame,
                 controller):
        """
        Base class for robot and simulated obstacles. Have the physical attributes of an agent.

        """
        super(Agent, self).__init__(px, py, vx, vy, radius, vpref, roll, pitch, yaw, gx, gy, sim_heading)

        assert isinstance(controller, BaseController)
        assert action_frame in ['body', 'global']

        self.dt = dt
        self.controller = controller
        self.controller.set_dt(dt)
        self.action_frame = action_frame

        self.other_agents = {}

    def get_full_state(self):
        return FullState(self.px, self.py, self.vx, self.vy, self.radius, self.vpref, self.theta[1], self.theta[0],
                         self.heading, self.gx, self.gy, self.sim_heading)

    def get_other_agents(self):
        return self.other_agents

    def get_controller_type(self):
        return self.controller.__str__()

    def set_other_agent(self, other_agent_name, other_agent_state):
        assert isinstance(other_agent_name, str)
        assert isinstance(other_agent_state, ObservableState)
        self.other_agents[other_agent_name] = other_agent_state

    def get_action(self):

        state = JointState(self.get_full_state(), self.get_other_agents())
        action_xy = self.controller.get_action(state, frame=self.action_frame)
        return action_xy

    def next_position(self, action, step_size):
        assert isinstance(action, ActionXY)
        if self.action_frame == 'global':
            px = self.px + action.vx * step_size
            py = self.py + action.vy * step_size
        else:
            dx = action.v * step_size * (np.cos(self.sim_heading) - np.sin(self.sim_heading))
            dy = action.v * step_size * (np.sin(self.sim_heading) + np.cos(self.sim_heading))
            px = self.px + dx
            py = self.py + dy

        return px, py

