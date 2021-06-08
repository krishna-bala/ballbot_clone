import numpy as np
import copy
import os
import tensorflow as tf

from ballbot_common.controllers.control_utils.cadrl_agent import CADRL_Agent
import ballbot_common.controllers.control_utils.network as network  # network.Config is defining a lot of stuff
from ballbot_common.controllers.control_utils.network import NetworkVP_rnn, Actions
from ballbot_common.controllers.base_controller import BaseController
from ballbot_common.utils.action import ActionRot, ActionXY

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)


class CADRL(BaseController):

    def __init__(self):
        super(CADRL, self).__init__()
        self.checkpoint_path = self.get_checkpoint()
        self.nn, self.actions = self.load_nn()
        self.ego_state = None
        self.other_states = None
        self.sim_heading = None

    def __str__(self):
        return 'CADRL'

    def load_nn(self):
        a = Actions()
        actions = a.actions
        num_actions = a.num_actions
        nn = NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
        wd = os.path.dirname(os.path.realpath(__file__))
        nn.simple_load(wd + '/control_utils/checkpoints/' + self.checkpoint_path)
        return nn, actions

    @staticmethod
    def get_checkpoint():
        date = '2021-04-16'
        time = '2021-04-16_14.28.59'
        checkpoint = '03395000'
        relative_path = date + '/' + time + '/' + 'network_' + checkpoint
        return relative_path

    def get_action(self, state, frame):
        """
        Predict an ActionXY, either in body frame or global frame.

        :param  state: Full State of Ego Agent and Observable State of Other Agents
        :type   state: JointState
        :param  frame: Either 'body' or 'global'
        :type   frame: string

        :return:    action_xy: Predicted action to take
        :type:      action_xy: ActionXY
        """

        assert frame in ['body', 'global']
        # Set ego state and state of other agents
        self.ego_state = state.self_state
        self.other_states = state.other_states

        if self.sim_heading is None:
            self.sim_heading = self.ego_state.get_heading()

        if self.at_goal():
            return ActionXY(0, 0)

        # Get policy predicted action
        action_rot = self.predict_action_rot()
        global_action_xy = self.action_rot_to_global_action_xy(action_rot)

        # Convert to ActionXY before updating sim_heading
        if frame == 'body':
            # Convert to body frame ActionXY
            action_xy = self.action_rot_to_body_action_xy(action_rot)
        else:
            # Convert to global frame ActionXY
            action_xy = global_action_xy

        self.sim_heading = self.wrap(self.sim_heading + action_rot.theta)

        return action_xy

    def predict_action_rot(self):
        """
        Gets the current state of ego agent and all other agents and predicts an ActionRot (v, delta_theta)
        :rtype: ActionRot
        """
        px, py = self.ego_state.get_position()
        vx, vy = self.ego_state.get_velocity()
        gx, gy = self.ego_state.get_goal()
        radius = self.ego_state.get_radius()
        vpref = self.ego_state.get_vpref()
        sim_heading = self.sim_heading

        # Porting to Everett's Agent Class
        ego_agent = CADRL_Agent(px, py, gx, gy, radius, vpref, sim_heading, 0)
        ego_agent.vel_global_frame = np.array([vx, vy])

        obs = self.get_observation(ego_agent)
        predictions = self.nn.predict_p(obs)[0]

        # raw_action is (c*vpref, delta_theta) where c = 0, 0.5, or 1 (from Actions())
        raw_action = copy.deepcopy(self.actions[np.argmax(predictions)])
        action_rot = ActionRot(vpref * raw_action[0], raw_action[1])

        return action_rot

    def get_observation(self, ego_agent):
        other_agents = self.get_cadrl_other_agents_from_obsverable_states()
        obs_with_id = ego_agent.observe(other_agents)
        obs_without_id = obs_with_id[1:]  # remove the first item which is id
        obs = np.expand_dims(obs_without_id, axis=0)
        return obs

    def get_cadrl_other_agents_from_obsverable_states(self):
        """
        Creates other_agents in the form of CADRL_Agent objects from ObservableState objects

        :return: other agents in CADRL_Agent form
        :type: list of CADRL_Agent
        """
        other_agents = []
        for i, agent_name in enumerate(self.other_states):
            other_agent = self.other_states[agent_name]
            index = i + 1
            x, y = other_agent.get_position()
            vx, vy = other_agent.get_velocity()
            radius = other_agent.get_radius()
            # Dummy goal values since it is unknown
            gx, gy = x, y
            # Dummy vpref since it is unknown
            pref_speed = np.linalg.norm(np.array([vx, vy]))
            heading = np.arctan2(vy, vx)
            other_agents.append(CADRL_Agent(x, y, gx, gy, radius, pref_speed, heading, index))

        return other_agents

    def action_rot_to_body_action_xy(self, action_rot):
        """

        :param action_rot: ActionRot
        :return: ActionXY in body frame
        """
        assert isinstance(action_rot, ActionRot)
        real_heading = self.ego_state.get_heading()
        sim_heading = self.sim_heading
        v = action_rot[0]
        delta_heading = action_rot[1]
        new_sim_heading = delta_heading + sim_heading

        global_vx = np.cos(new_sim_heading) * v
        global_vy = np.sin(new_sim_heading) * v

        body_vx = np.cos(real_heading) * global_vx + np.sin(real_heading) * global_vy
        body_vy = -np.sin(real_heading) * global_vx + np.cos(real_heading) * global_vy

        return ActionXY(body_vx, body_vy)

    def action_rot_to_global_action_xy(self, action_rot):
        """

        :param action_rot: ActionRot
        :return: ActionXY in global frame
        """
        assert isinstance(action_rot, ActionRot)
        sim_heading = self.sim_heading
        v = action_rot[0]
        delta_heading = action_rot[1]
        new_sim_heading = delta_heading + sim_heading

        global_vx = np.cos(new_sim_heading) * v
        global_vy = np.sin(new_sim_heading) * v

        return ActionXY(global_vx, global_vy)

    def set_sim_heading(self, sim_heading):
        self.sim_heading = sim_heading

    @staticmethod
    def wrap(angle):
        while angle >= np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
