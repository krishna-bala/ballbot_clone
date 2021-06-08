from tf import transformations
from ballbot_common.utils.action import ActionXY
import numpy as np


class ObservableState(object):
    def __init__(self, px, py, vx, vy, radius):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.radius = radius

    def get_position(self):
        return self.px, self.py

    def set_position(self, pos):
        self.px, self.py = pos[0], pos[1]

    def get_velocity(self):
        return self.vx, self.vy

    def set_velocity(self, vel):
        self.vx, self.vy = vel[0], vel[1]

    def get_radius(self):
        return self.radius


class ROSState(ObservableState):
    def __init__(self, px, py, vx, vy, radius, vpref, roll, pitch, yaw):
        super(ROSState, self).__init__(px, py, vx, vy, radius)
        self.vpref = vpref

        # theta_x, theta_y
        self.theta = [pitch, roll]
        self.heading = yaw

    def set_state(self, px, py, vx, vy, roll, pitch, yaw):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.theta = []

    def get_vpref(self):
        return self.vpref

    def get_heading(self):
        return self.heading

    def get_theta(self):
        return self.theta

    def get_observable_state(self):
        return ObservableState(self.px, self.py, self.vx, self.vy, self.radius)

    def next_position(self, action, step_size):
        assert isinstance(action, ActionXY)
        px = self.px + action.vx * step_size
        py = self.py + action.vy * step_size

        return px, py


class FullState(ROSState):
    def __init__(self, px, py, vx, vy, radius, vpref, roll, pitch, yaw, gx, gy, sim_heading):
        super(FullState, self).__init__(px, py, vx, vy, radius, vpref, roll, pitch, yaw)
        self.gx = gx
        self.gy = gy
        self.sim_heading = sim_heading

        # theta_dot_x, theta_dot_y
        self.theta_dot = [0, 0]

    def get_goal(self):
        return self.gx, self.gy

    def get_sim_heading(self):
        return self.sim_heading

    def get_theta_dot(self):
        return self.theta_dot

    # def set(self, px, py, gx, gy, vx, vy, roll, pitch, yaw, sim_heading, radius=None, vpref=None):
    #     self.px = px
    #     self.py = py
    #     self.gx = gx
    #     self.gy = gy
    #     self.vx = vx
    #     self.vy = vy
    #     self.roll = roll
    #     self.pitch = pitch
    #     self.heading = yaw
    #     self.sim_heading = sim_heading

        # if radius is not None:
        #     self.radius = radius
        # if vpref is not None:
        #     self.vpref = vpref

    def set_radius(self, radius):
        self.radius = radius

    def set_goal(self, goal):
        self.gx, self.gy = goal[0], goal[1]

    def set_vpref(self, vpref):
        self.vpref = vpref

    def set_theta(self, theta):
        self.theta = theta

    def set_theta_dot(self, theta_dot):
        self.theta_dot = theta_dot

    def set_heading(self, heading):
        self.heading = heading

    def set_sim_heading(self, sim_heading):
        self.sim_heading = sim_heading


class JointState(object):
    def __init__(self, self_state, other_states):
        assert isinstance(self_state, FullState)

        for agent_name in other_states:
            assert isinstance(other_states[agent_name], ObservableState)

        self.self_state = self_state
        self.other_states = other_states


def get_heading(orientation):
    q = orientation
    _, _, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw
