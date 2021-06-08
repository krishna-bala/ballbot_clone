import rospy
from tf import transformations
import rospkg
import yaml
import os
from datetime import datetime

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState

from ballbot_sim.utils.sim_util import get_names, load_cost_params

from ballbot_common.utils.state import ROSState
from ballbot_common.utils.agent import Agent
from ballbot_common.controllers.controller_factory import controller_factory
from ballbot_common.controllers.MPC import MPC
from ballbot_common.controllers.ORCA import ORCA
from ballbot_common.controllers.control_utils.traj_gen import TrajGen
from ballbot_common.controllers.control_utils.traj_eval import TrajEval
from ballbot_common.data_record.agent_data_record import AgentDataRecord
from ballbot_common.data_record.traj_gen_record import TrajGenRecord


class SimMasterControlNode(object):
    """
    Master controller for all agents. Each agent has its own individual controller object.

    Subscribers:
        /all_agents/model_states
        /all_agents/finished
        /agent_name/goal (multiple)

    Publishers:
        /cmd_vel (only used for ballbot agent)

    Services:
        /gazebo/set_model_state (used for all spheres aka other agents)

    Controller types: CADRL, ORCA, PID, MPC
    MPC controller has additional steps to generate and evaluate trajectories.

    Saves data_record after each experiment run.
    """

    def __init__(self, dt, rollout_horizon, num_rollouts, span, update_rate, num_trials):

        # Wait until all agents are published before initializing the master controller
        rospy.wait_for_message('/all_agents/published', Bool)
        self.dt = dt  # dt used for MPC rollouts, PID error, and other controller functions
        self.update_rate = update_rate  # Used for stepping the spheres in gazebo
        self.rollout_horizon = rollout_horizon
        self.num_rollouts = num_rollouts
        self.span = span
        self.agent_names = get_names()
        self.ROS_states = {}
        # Make sure all agent states and goals have been received from the subscriber callback before initializing
        self.ROS_states_set = False
        self.agents = {}
        self.agents_vpref = {}
        self.agents_radius = {}
        self.goals = {}
        self.actions = {}
        self.data_recorder = None
        self.traj_gen_recorder = None

        self.trial = 0
        self.num_trials = num_trials
        self.experiment_finished = False
        self.reset_experiment = True
        self.saving_data = False
        self.setup_pub_sub_srv()
        rospack = rospkg.RosPack()
        self.pack_path = rospack.get_path('ballbot_sim')
        self.save_path = get_save_path(self.pack_path)
        self.initialize_data_recording()

    # noinspection PyAttributeOutsideInit
    def setup_pub_sub_srv(self):
        """
        Subscribers:
            /all_agents/model_states
            /all_agents/finished
            /agent_name/goal (multiple)

        Publishers:
            /cmd_vel (only used for ballbot agent)

        Services:
            /gazebo/set_model_state (used for all spheres aka other agents)

        """
        # Publisher
        self.ballbot_action_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        self.goal_subs = []
        for agent_name in self.agent_names:
            # Subscribe to each goal topic
            self.goal_subs.append(rospy.Subscriber('/' + agent_name + '/goal', Point, self.get_goal,
                                                   callback_args=agent_name))

            # Set the agent vpref and radius from param server
            self.agents_vpref[agent_name] = rospy.get_param('/sim_experiment_node/' + agent_name + '/vpref')
            self.agents_radius[agent_name] = rospy.get_param('/sim_experiment_node/' + agent_name + '/radius')

        self.all_agents_sub = rospy.Subscriber('/all_agents/model_states', ModelStates, self.get_all_states)
        self.all_agents_done_sub = rospy.Subscriber('/all_agents/finished', Bool, self.all_agents_finished_callback)

        # Todo: Is it enough to use rospy.wait_for_message? So far, yes.
        # self.all_agents_initialized_sub = rospy.Subscriber('/all_agents/initialized', Bool,
        #                                                    self.all_agents_finished_callback)

        # Service
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def initialize_data_recording(self):
        self.traj_gen_recorder = TrajGenRecord(self.save_path, 'ballbot', self.num_rollouts)
        self.data_recorder = AgentDataRecord(self.save_path)

    def get_goal(self, msg, agent_name):
        """
        Subscriber callback for /agent_name/goal. Stores the goal for each agent in the goals dict.

        :param msg: goal point for each agent
        :type msg: Point
        :param agent_name: agent name
        :type agent_name: str
        """
        if agent_name not in self.goals:
            # Todo: Make a goal class and use setter
            self.goals[agent_name] = msg
        else:
            # Only update when our message changes to a new goal
            # Note: we are not changing goals right now so this never changes.
            if self.goals[agent_name] != msg:
                self.goals[agent_name] = msg

    def get_all_states(self, msg):
        """
        Subscriber callback for /all_agents/model_states. Creates a ROSState object that includes the radius, vpref,
        and heading of the agent and stores it in ROS_states dict.

        :param msg: Name, Pose, and Twist info for all agents
        :type msg: ModelStates
        """
        agent_names = msg.name
        poses = msg.pose
        twists = msg.twist
        for i, agent_name in enumerate(agent_names):
            px, py = poses[i].position.x, poses[i].position.y
            vx, vy = twists[i].linear.x, twists[i].linear.y
            q = poses[i].orientation
            roll, pitch, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            vpref = self.agents_vpref[agent_name]
            radius = self.agents_radius[agent_name]
            ROS_state = ROSState(px, py, vx, vy, radius, vpref, roll, pitch, yaw)
            self.ROS_states[agent_name] = ROS_state

        # Update flag so we can begin initializing agent objects
        self.ROS_states_set = True

    def initialize_agents(self):
        """
        Initializes all agents as Agent object from their ROSState. Stores each object in the agents dict. Each Agent
        oject has a controller that determines what the next action will be give the ego agent's FullState and the
        other agents' ObservableState.

        Reset Experiment flag is toggled off once the agents have been initialized.
        """

        self.ROS_states_set = False
        # Wait for the /all_agents/model_states callback to set the states for all agents
        while not self.ROS_states_set:
            continue

        for agent_name in self.ROS_states:
            # Wait for the /agent_name/goal callback to set the goal for each agent
            while agent_name not in self.goals:
                continue

        for agent_name in self.ROS_states:
            ros_state = self.ROS_states[agent_name]
            goal = self.goals[agent_name]

            px, py = ros_state.get_position()
            vx, vy = ros_state.get_velocity()
            radius = ros_state.get_radius()
            pitch, roll = ros_state.get_theta()
            yaw = ros_state.get_heading()

            # Todo: Make a goal class and use getter
            gx, gy = goal.x, goal.y
            vpref = ros_state.get_vpref()
            dt = self.dt
            # Sim Heading is same as real heading before any actions are taken
            sim_heading = yaw

            # Commands are issued in body frame for ballbot and global frame for sphere obstacles
            if agent_name == 'ballbot':
                frame = 'body'
            else:
                frame = 'global'

            controller_type = rospy.get_param('/sim_experiment_node/' + agent_name + '/controller')
            controller = controller_factory[controller_type]()

            # Additional steps for initializing ORCA
            if isinstance(controller, ORCA):
                controller.set_radius(radius)

            # Additional steps for initializing MPC controller
            if isinstance(controller, MPC):
                rollout_type = rospy.get_param('/sim_experiment_node/' + agent_name + '/rollout_policy')
                rollout_policy = controller_factory[rollout_type]()

                rollout_horizon = self.rollout_horizon
                num_rollouts = self.num_rollouts
                span = self.span

                traj_gen = TrajGen(dt, frame, rollout_policy, rollout_horizon, num_rollouts, span)
                controller.set_traj_gen(traj_gen)
                controller.set_traj_gen_record(self.traj_gen_recorder)

                cost_params = load_cost_params(agent_name)
                num_obs = len(self.ROS_states) - 1

                traj_eval = TrajEval(cost_params, num_obs, vpref, dt, rollout_horizon, radius)
                controller.set_traj_eval(traj_eval)

            # Create agent and store it in class member variable (dict)
            self.agents[agent_name] = Agent(px, py, vx, vy, radius, vpref, roll, pitch, yaw, gx, gy, sim_heading, dt,
                                            frame, controller)

        # Toggle to false now that the experiment has been reset
        self.reset_experiment = False

    def set_agents(self):
        """
        Update all Agent objects with information from ROS.
        """
        for agent_name in self.ROS_states:
            # Get state info from ROS callback
            ros_state = self.ROS_states[agent_name]
            pos = ros_state.get_position()
            vel = ros_state.get_velocity()
            theta = ros_state.get_theta()

            prev_theta = self.agents[agent_name].get_theta()
            theta_dot_x = (theta[0] - prev_theta[0]) / self.update_rate
            theta_dot_y = (theta[1] - prev_theta[1]) / self.update_rate
            theta_dot = [theta_dot_x, theta_dot_y]

            heading = ros_state.get_heading()

            # Get goal info from ROS callback
            goal = self.goals[agent_name].x, self.goals[agent_name].y

            # Update agents with ROS info
            self.agents[agent_name].set_position(pos)
            self.agents[agent_name].set_goal(goal)
            self.agents[agent_name].set_velocity(vel)
            self.agents[agent_name].set_theta(theta)
            self.agents[agent_name].set_theta_dot(theta_dot)
            self.agents[agent_name].set_heading(heading)
            # Note: sim_heading is not updated by ROS, only the policy/controller

            # Todo: Is there a better way than nested for loop?
            # Set other agents info for curr agent
            for other_agent_name in self.ROS_states:
                # Check it's not curr agent
                if other_agent_name != agent_name:
                    # Get observable state of other agent and update curr agent with info
                    other_agent_state = self.ROS_states[other_agent_name].get_observable_state()
                    self.agents[agent_name].set_other_agent(other_agent_name, other_agent_state)

    def get_agents(self):
        """
        Get all Agent objects.

        :return: Dictionary of Agent objects, where string name is key and Agent object is value.
        :type: dict(str, Agent)
        """
        return self.agents

    def set_action(self, agent_name, action_xy):
        """

        :param agent_name: Name of agent
        :type agent_name: str
        :param action_xy: Action that agent will take during execution.
        :type action_xy: ActionXY (global frame if sphere, body frame if ballbot)
        """
        self.actions[agent_name] = action_xy

    def all_agents_finished_callback(self, msg):
        """
        Subscriber callback to /all_agents/finished

        Updates the experiment status to True if all agents are finished.
        Only updates if we are not in the process of saving data_record.

        :param msg: callback for /all_agents/finished
        :type msg: Bool
        """
        if not self.saving_data:
            self.experiment_finished = msg.data

    def execute_actions(self):
        """
        Execute the actions for all agents if the experiment is not finished.
        Save data_record and set the reset experiment flag to true if the experiment is finished,
        """
        if not self.experiment_finished:
            agents = self.get_agents()
            self.data_recorder.add_data(agents)

            for agent_name in self.actions:
                action_xy = self.actions[agent_name]

                # Ballbot actions are published to /cmd_vel
                if agent_name == 'ballbot':
                    msg = Twist()
                    msg.linear.x = action_xy.vx
                    msg.linear.y = action_xy.vy
                    self.ballbot_action_pub.publish(msg)
                else:
                    # All other agents are updated with /gazebo/set_model_state service
                    next_pos = self.ROS_states[agent_name].next_position(action_xy, step_size=self.update_rate)
                    next_state = ModelState()
                    next_state.model_name = agent_name
                    next_state.pose.position.x = next_pos[0]
                    next_state.pose.position.y = next_pos[1]
                    next_state.twist.linear.x = action_xy.vx
                    next_state.twist.linear.y = action_xy.vy
                    self.set_model_state(next_state)
        else:
            self.saving_data = True
            save_yamls(self.pack_path, self.save_path)
            self.data_recorder.save_data(self.trial)
            self.traj_gen_recorder.save_data(self.trial)
            self.data_recorder.clear_data()
            self.traj_gen_recorder.clear_data()
            self.saving_data = False

            # Waits for the callback to update the experiment status;
            # ensures we only move the trial counter by 1 and that
            while self.experiment_finished:
                continue

            self.trial += 1
            if self.trial >= self.num_trials:
                rospy.signal_shutdown("Completed all trials.")

            # Toggle the reset flag so we re-initialize the states of all agents.
            self.reset_experiment = True

    def get_reset_experiment(self):
        return self.reset_experiment


def save_yamls(pack_path, save_path):
    """
    Save the yamls that were used to initialize the experiment.

    :param pack_path: the file path for the current ROS package, ballbot_sim
    :param save_path: the save path for where we want to save the yamls
    """
    yaml_dir = pack_path + '/launch/params/'

    with open(yaml_dir + 'ballbot_cost_params.yaml', 'r') as read_file:
        cost_params = yaml.load(read_file, Loader=yaml.FullLoader)
        with open(save_path + 'cost_params.yaml', 'w') as write_file:
            yaml.dump(cost_params, write_file)

    config_name = rospy.get_param('/sim_experiment_node/experiment_config')
    with open(yaml_dir + 'agents_config_' + config_name + '.yaml', 'r') as read_file:
        config = yaml.load(read_file, Loader=yaml.FullLoader)
        with open(save_path + config_name + '.yaml', 'w') as write_file:
            yaml.dump(config, write_file)


def get_save_path(pack_path):
    """
    Helper function to set the save path relative to the ballbot_sim ROS package.

    :return: Path to where we will be saving experiments
    :type: str
    """
    base_path = pack_path + '/log/exp-data/'

    exp_type = rospy.get_param('/sim_experiment_node/experiment_config')

    compound_path = base_path + exp_type + '/'

    if not os.path.exists(compound_path):
        os.mkdir(compound_path)

    today = datetime.today()
    today = today.strftime("%Y-%m-%d/")

    compound_path += today

    if not os.path.exists(compound_path):
        os.mkdir(compound_path)

    time = datetime.now()
    time = time.strftime("%H.%M.%S/")

    compound_path += time

    if not os.path.exists(compound_path):
        os.mkdir(compound_path)

    return compound_path
