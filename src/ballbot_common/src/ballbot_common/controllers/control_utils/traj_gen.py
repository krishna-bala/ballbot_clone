import numpy as np
import copy
from time import time

from geometry_msgs.msg import Twist, Pose2D
from gazebo_msgs.msg import ModelState, ModelStates

from ballbot_common.controllers.PID import PID
from ballbot_common.controllers.CADRL import CADRL
from ballbot_common.controllers.base_controller import BaseController
from ballbot_common.utils.action import ActionXY, ActionRot


# noinspection PyAttributeOutsideInit
class TrajGen(object):

    def __init__(self, dt, action_frame, rollout_policy,
                 rollout_horizon, num_rollouts, span):

        assert isinstance(rollout_policy, CADRL)
        self.dt = dt
        self.rollout_policy = rollout_policy
        self.rollout_policy.set_dt(dt)
        self.action_frame = action_frame

        self.rollout_horizon = rollout_horizon
        self.num_rollouts = num_rollouts
        assert 0 < span <= 2 * np.pi
        self.span = span
        if self.span == 2 * np.pi:
            self.span = 2 * np.pi - ((2 * np.pi) / num_rollouts)

        self.ego_state = None
        self.other_states = None
        self.joint_state = None
        self.sim_heading = None

    def __str__(self):
        return self.rollout_policy.__str__()

    def generate_rollouts(self, state):
        self.joint_state = state
        self.ego_state = state.self_state
        self.other_states = state.other_states
        waypoints = self.get_waypoints()
        ref_trajs = []
        for goal in waypoints:
            traj = self.generate_trajectory(goal)
            ref_trajs.append(traj)
        return ref_trajs

    def get_waypoints(self):

        waypoints = []
        px, py = self.ego_state.get_position()
        gx, gy = self.ego_state.get_goal()
        vpref = self.ego_state.get_vpref()
        if self.num_rollouts > 1:
            for i in range(self.num_rollouts):
                theta = -(self.span / 2.0) + i * self.span / (self.num_rollouts - 1)
                sim_heading = self.ego_state.get_sim_heading()
                new_gx = px + (vpref * 10 * self.rollout_horizon) * np.cos(sim_heading + theta)
                new_gy = py + (vpref * 10 * self.rollout_horizon) * np.sin(sim_heading + theta)
                waypoints.append((new_gx, new_gy))
        else:
            dx = gx - px
            dy = gy - py
            heading_to_goal = np.arctan2(dy, dx)
            new_gx = px + (vpref * 10 * self.rollout_horizon) * np.cos(heading_to_goal)
            new_gy = py + (vpref * 10 * self.rollout_horizon) * np.sin(heading_to_goal)
            waypoints.append((new_gx, new_gy))

        return waypoints

    def generate_trajectory(self, goal, other_agents_policy='linear'):
        """
        Traj format:
          [theta_x, theta_dot_x, global_action.vx, theta_y, theta_dot_y, global_action.vy, px, py,
            waypoint.x, waypoint.y, other_agent1_px, other_agent1_vx, other_agent1_py, other_agent1_vy, other_agent2...]

        :param goal: waypoint
        :param other_agents_policy: linear
        :return: traj
        """

        # Copy the joint state of ego agent and other agents so we don't make any modifications during rollouts.
        joint_state = copy.deepcopy(self.joint_state)
        ego_ref = joint_state.self_state               # ego_ref is a reference to the copied joint_state attr
        ego_ref.set_goal(goal)                         # set the waypoint as the goal
        other_states_ref = joint_state.other_states    # other_states_copy is a reference to the copied joint_state attr

        # Reset rollout to match sim heading of ego agent instead of previous trajectory rollout
        self.rollout_policy.set_sim_heading(ego_ref.get_sim_heading())

        # Initialize our return value
        traj = []

        # Initialize trajectory with current states of all agents
        px, py = self.ego_state.get_position()
        vx, vy = self.ego_state.get_velocity()
        gx, gy = goal[0], goal[1]
        pool = [0, 0, vx, 0, 0, vy, px, py, gx, gy]
        for agent_name in self.other_states:
            other_agent = self.other_states[agent_name]
            px, py = other_agent.get_position()
            vx, vy = other_agent.get_velocity()
            agent = [px, py, vx, vy]
            pool += agent
        traj.append(pool)

        time_steps = int(self.rollout_horizon / self.dt)
        # Todo: Can improve runtime if we only calculate once for all trajectories.
        other_states_propagated = self.predict_other_agents_traj(time_steps, other_states_ref,
                                                                 policy=other_agents_policy)

        # Step all agents for each time step and append states at each step to traj
        for k in range(time_steps):

            # Set other states to state at current time step
            joint_state.other_states = other_states_propagated[k]
            # Get global action for ego agent
            global_action_xy = self.rollout_policy.get_action(joint_state, frame='global')

            next_state = self.step_dynamics(ego_ref, global_action_xy)

            next_theta = [next_state[0], next_state[3]]
            next_theta_dot = [next_state[1], next_state[4]]
            next_vel = [next_state[2], next_state[5]]

            # Step position with the resulting global velocity from ballbot dynamics
            px, py = ego_ref.get_position()
            next_px = px + next_vel[0] * self.dt
            next_py = py + next_vel[1] * self.dt
            next_pos = [next_px, next_py]

            # Update the joint_state.self_state info
            ego_ref.set_position(next_pos)
            ego_ref.set_velocity(next_vel)
            ego_ref.set_theta(next_theta)
            ego_ref.set_theta_dot(next_theta_dot)

            # Note: Add the reference action and the next position to our reference trajectory.
            pool = [0, 0, global_action_xy.vx, 0, 0, global_action_xy.vy, next_px, next_py, gx, gy]

            # Add the stepped position and velocity of other agents to reference trajectory
            other_states_curr_step = other_states_propagated[k + 1]
            for other_agent_name in other_states_curr_step:
                other_agent_curr_step = other_states_curr_step[other_agent_name]
                pos = other_agent_curr_step.get_position()
                vel = other_agent_curr_step.get_velocity()
                pool += pos + vel
            traj.append(pool)

        return traj

    def predict_other_agents_traj(self, time_steps, other_states_copy, policy):
        """
        Propagates other agents and stores (time_steps+1) states to be used for predicting the ego_agent's next action

        :param time_steps: number of steps to propagate forward
        :type time_steps: int
        :param other_states_copy: copy of joint_state.other_states
        :type other_states_copy: list of dict(str,ObservableState)
        :param policy: how agents will be propagated
        :type policy: str

        :return: other_states_propagated
        :type: list, length (time_steps + 1), of dict(str:ObservableState) (aka joint_states.other_states)
        """

        # Initialize a list with other_states_copy
        other_states_propagated = [other_states_copy]

        # Propagate other states and save in array for use with rollout_policy.get_action()
        for k in range(time_steps):
            next_other_states = {}
            for i, agent_name in enumerate(other_states_copy):
                # Get other agent
                other_agent = other_states_copy[agent_name]
                # Predict next pos and next vel
                next_pos, next_vel = self.predict_other_agent_step(other_agent, agent_policy=policy)
                # Update other states copy to next pos and next vel
                other_states_copy[agent_name].set_position(next_pos)
                other_states_copy[agent_name].set_velocity(next_vel)
                # Add next state to the dict of next other states
                next_other_states[agent_name] = other_states_copy[agent_name]
            # Append our dict of other states to the list of propagated other states
            other_states_propagated.append(next_other_states)

        return other_states_propagated

    def predict_other_agent_step(self, other_agent_copy, agent_policy='linear'):
        px, py = other_agent_copy.get_position()
        vx, vy = other_agent_copy.get_velocity()
        if agent_policy == 'linear':
            next_px = px + vx * self.dt
            next_py = py + vy * self.dt
        else:
            raise NotImplementedError
        return (next_px, next_py), (vx, vy)

    def step_dynamics(self, ego_ref, action):

        # Get current (global) state info for ego agent
        pos = ego_ref.get_position()
        vel = ego_ref.get_velocity()
        theta = ego_ref.get_theta()
        theta_dot = ego_ref.get_theta_dot()

        # State (global):
        #   [theta_x, theta_dot_x, vx, theta_y, theta_dot_y, vy, x, y]
        state = [theta[0], theta_dot[0], vel[0], theta[1], theta_dot[1], vel[1], pos[0], pos[1]]
        S_curr = np.array(state)

        # Reference input (global)
        U = np.array([0, 0, action[0], 0, 0, action[1]])

        # Integrate with ballbot dynamics
        S_next = self.integrator(S_curr, U)

        return S_next

    def integrator(self, S, U):
        M = 4
        dt_ = float(self.dt) / M
        S_next = np.array(S)
        for i in range(M):
            k1 = dt_ * self.state_dot(S, U)
            k2 = dt_ * self.state_dot(S + (0.5 * k1), U)
            k3 = dt_ * self.state_dot(S + (0.5 * k2), U)
            k4 = dt_ * self.state_dot(S + k3, U)
            S_next += (k1 + 2 * k2 + 2 * k3 + k4) / 6
        return S_next

    @staticmethod
    def state_dot(S0, U):
        S_dot = np.array(S0)
        S_dot[0] = S0[1]
        S_dot[1] = ((-38.73 * S0[0]) + (-11.84 * S0[1]) + (-6.28 * S0[2]) +
                    (51.61 * U[0]) + (11.84 * U[1]) + (6.28 * U[2]))

        S_dot[2] = ((13.92 * S0[0]) + (2.0 * S0[1]) + (1.06 * S0[2]) +
                    (-8.72 * U[0]) + (-2.0 * U[1]) + (-1.06 * U[2]))

        S_dot[3] = S0[4]
        S_dot[4] = ((-38.54 * S0[3]) + (-11.82 * S0[4]) + (-6.24 * S0[5]) +
                    (51.36 * U[3]) + (11.82 * U[4]) + (6.24 * U[5]))

        S_dot[5] = ((14.00 * S0[3]) + (2.03 * S0[4]) + (1.07 * S0[5]) +
                    (-8.81 * U[3]) + (-2.03 * U[4]) + (-1.07 * U[5]))
        return S_dot

    def get_next_pos(self, action_xy, pos):
        assert isinstance(action_xy, ActionXY)
        px = pos[0] + action_xy.vx * self.dt
        py = pos[1] + action_xy.vy * self.dt
        next_pos = (px, py)
        return next_pos
