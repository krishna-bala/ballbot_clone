import numpy as np
from tf import transformations


class BaseController(object):

    def __init__(self):
        self.dt = None
        self.goal_thresh = 0.5
        self.ego_state = None
        self.other_states = None

    def get_action(self, state, frame):
        raise NotImplementedError

    def at_goal(self):
        pos = self.ego_state.get_position()
        goal = self.ego_state.get_goal()
        radius = self.ego_state.get_radius()
        dist_to_goal = (pos[0] - goal[0], pos[1] - goal[1])
        if np.linalg.norm(dist_to_goal) - radius < self.goal_thresh:
            return True
        return False

    def set_dt(self, dt):
        self.dt = dt