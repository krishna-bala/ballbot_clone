#!/usr/bin/env python
import numpy as np

# TODO: Add rvo to setup instructions for ROS pkg
import rvo2

from ballbot_common.utils.action import ActionXY
from ballbot_common.controllers.base_controller import BaseController


class ORCA(BaseController):

    def __init__(self):

        super(ORCA, self).__init__()
        self.safety_space = 0
        self.neighbor_dist = 10
        self.max_neighbors = 10
        self.time_horizon = 5
        self.time_horizon_obst = 5
        self.max_speed = 1
        self.sim = None
        self.radius = None
        self.ego_state = None
        self.other_states = None
        self.sim_heading = None

    def set_radius(self, radius):
        self.radius = radius

    def get_action(self, state, frame):

        assert frame in ['body', 'global']
        # Set ego state and state of other agents
        self.ego_state = state.self_state
        self.other_states = state.other_states

        if self.at_goal():
            return ActionXY(0, 0)

        self_pos = self.ego_state.get_position()
        self_vel = self.ego_state.get_velocity()
        vpref = self.ego_state.get_vpref()
        goal = self.ego_state.get_goal()

        if self.sim is not None and self.sim.getNumAgents() != len(self.other_states) + 1:
            del self.sim
            self.sim = None

        if self.sim is None:
            self.sim = rvo2.PyRVOSimulator(self.dt, self.neighbor_dist, self.max_neighbors, self.time_horizon,
                                           self.time_horizon_obst, self.radius, self.max_speed)

            self.sim.addAgent(self_pos, self.neighbor_dist, self.max_neighbors, self.time_horizon,
                              self.time_horizon_obst, self.radius + self.safety_space,
                              vpref, self_vel)

            for agent_name in self.other_states:
                other_agent = self.other_states[agent_name]
                other_agent_pos = other_agent.get_position()
                other_agent_vel = other_agent.get_velocity()

                self.sim.addAgent(other_agent_pos, self.neighbor_dist, self.max_neighbors, self.time_horizon,
                                  self.time_horizon_obst, self.radius + self.safety_space,
                                  self.max_speed, other_agent_vel)
        else:
            self.sim.setAgentPosition(0, self_pos)
            self.sim.setAgentVelocity(0, self_vel)
            for i, agent_name in enumerate(self.other_states):
                other_agent = self.other_states[agent_name]
                other_agent_pos = other_agent.get_position()
                other_agent_vel = other_agent.get_velocity()
                self.sim.setAgentPosition(i + 1, other_agent_pos)
                self.sim.setAgentVelocity(i + 1, other_agent_vel)

        # Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        velocity = np.array((goal[0] - self_pos[0], goal[1] - self_pos[1]))
        speed = np.linalg.norm(velocity)
        pref_vel = velocity / speed if speed > 1 else velocity

        self.sim.setAgentPrefVelocity(0, tuple(pref_vel))
        for i in range(len(self.other_states)):
            # unknown goal position of other humans
            self.sim.setAgentPrefVelocity(i + 1, (0, 0))

        self.sim.doStep()

        self_vel = self.sim.getAgentVelocity(0)
        global_action_xy = ActionXY(self_vel[0], self_vel[1])

        if frame == 'body':
            action_xy = self.action_global_xy_to_body_xy(global_action_xy)
        else:
            action_xy = global_action_xy

        return action_xy

    def action_global_xy_to_body_xy(self, global_action_xy):
        heading = self.ego_state.get_heading()
        body_vx = np.cos(heading) * global_action_xy.vx + np.sin(heading) * global_action_xy.vy
        body_vy = -np.sin(heading) * global_action_xy.vx + np.cos(heading) * global_action_xy.vy

        return ActionXY(body_vx, body_vy)
