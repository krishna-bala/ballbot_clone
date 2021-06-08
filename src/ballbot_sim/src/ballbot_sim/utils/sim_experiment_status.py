import rospy
import numpy as np
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from ballbot_sim.srv import Initialize, InitializeResponse


# noinspection PyAttributeOutsideInit
class SimExperimentStatus(object):
    """
    Subscribes to agents' model states (/agent_name/model_state) and goal (/agent_name/goal).
    Checks if agents are at goal.
    Publishes that experiment is finished if all agents at goal or if time limit reached.

    """

    def __init__(self, names):

        self.exp_time_limit = 30
        self.setup_srv()
        self.agent_names = names

        self.agents_model_state = {}
        self.agents_goal = {}
        self.agents_radius = {}
        self.agents_finished_status = {}
        self.experiment_finished_status = False

        self.goal_thresh = 0.5

        self.setup_subs_pubs()

        # Service for initializing the experiment
        self.server = rospy.Service('initialize_experiment', Initialize, self.handle_initialize_experiment)

    def setup_srv(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def setup_subs_pubs(self):
        """
        Initialize all the publishers and subscribers. Also get the radius of each agent from param server.

        Publisher:
            /all_agents/finished
        Subscribers:
            /agent_name/model_state (multiple)
            /agent_name/goal (multiple)

        """
        self.agent_model_state_subs = []
        self.agent_goal_subs = []
        self.agent_finished_pubs = {}
        for agent_name in self.agent_names:
            topic_ns = '/' + agent_name
            model_state_topic = topic_ns + '/model_state'
            goal_topic = topic_ns + '/goal'

            radius = rospy.get_param('/sim_experiment_node/' + agent_name + '/radius')
            self.agents_radius[agent_name] = radius

            # Waits for sim_states_node to start publishing states and goals
            rospy.wait_for_message(model_state_topic, ModelState)
            rospy.wait_for_message(goal_topic, Point)

            self.agent_model_state_subs.append(rospy.Subscriber('/' + agent_name + '/model_state', ModelState,
                                                                self.set_agent_state, callback_args=agent_name))
            self.agent_goal_subs.append(rospy.Subscriber('/' + agent_name + '/goal', Point,
                                                         self.set_agent_goal, callback_args=agent_name))

        self.all_agents_finished_pub = rospy.Publisher('/all_agents/finished', Bool, queue_size=10)

    def set_agent_state(self, msg, agent_name):
        """
        Updates the state of each agent using data_record from /agent_name/model_state

        :param msg: Name, Pose, and Twist of each agent.
        :type msg: ModelState()
        :param agent_name: Name of agent
        :type agent_name: str
        """
        state = ModelState()
        state.model_name = msg.model_name
        state.pose = msg.pose
        state.twist = msg.twist
        self.agents_model_state[agent_name] = state

    def set_agent_goal(self, msg, agent_name):
        """
        Sets the appropriate goal for each agent. Checks if agent is at the goal and updates the status of the agent.

        :param msg: Goal location of agent.
        :type msg: Point
        :param agent_name: Agent name.
        :type agent_name: str
        """
        goal = Point()
        goal.x = msg.x
        goal.y = msg.y
        self.agents_goal[agent_name] = goal

        curr_state = self.agents_model_state[agent_name]
        radius = self.agents_radius[agent_name]

        px, py = curr_state.pose.position.x, curr_state.pose.position.y
        gx, gy = goal.x, goal.y
        dist_to_goal = np.linalg.norm((gx - px, gy - py))

        if dist_to_goal - radius < self.goal_thresh:
            self.agents_finished_status[agent_name] = True
        else:
            self.agents_finished_status[agent_name] = False

    def update_experiment_status(self):
        """
        Updates the status of the experiment; checks if all agents are at the goal or if the experiment has timed out.
        """
        all_agents_finished = True

        # Check if all agents are at their goal.
        for agent_name in self.agent_names:
            if not self.agents_finished_status[agent_name]:
                all_agents_finished = False

        # Check if the experiment has timed out.
        curr_time = time.time()
        experiment_timed_out = False
        if curr_time - self.exp_start_time > self.exp_time_limit:
            experiment_timed_out = True

        if all_agents_finished or experiment_timed_out:
            self.experiment_finished_status = True
        else:
            self.experiment_finished_status = False

        # Publish if all agents have finished so their controllers can stop executing actions
        self.publish_all_agents_finished(self.experiment_finished_status)

    def publish_all_agents_finished(self, status):
        self.all_agents_finished_pub.publish(Bool(status))

    def get_experiment_status(self):
        return self.experiment_finished_status

    def handle_initialize_experiment(self, foo):
        """
        Service handler to initialize the experiment. Stages all agents outside of normal arena and then moves them
        to their appropriate starting position. Starts timing the experiment once they have moved.

        :param foo: dummy variable for service request
        :return: True
        :type: InitializeResponse
        """

        # Wait for all agents to come to a rest
        rospy.sleep(5.0)

        # Move all agents to a staging area to ensure no collisions
        for i, agent_name in enumerate(self.agent_names):
            pose = ModelState()
            pose.model_name = agent_name
            pose.pose.position.x = -10 - 2 * i
            pose.pose.position.y = -10 - 2 * i
            pose.twist.linear.x = 0
            pose.twist.linear.y = 0
            self.set_model_state(pose)

        # Pause
        # rospy.sleep(2.0)

        # Move all agents to their start position
        for agent_name in self.agent_names:
            pose = ModelState()
            start_pos = rospy.get_param('/sim_experiment_node/' + agent_name + '/start')
            pose.model_name = agent_name
            pose.pose.position.x = start_pos[0]
            pose.pose.position.y = start_pos[1]
            pose.twist.linear.x = 0
            pose.twist.linear.y = 0
            self.set_model_state(pose)

            # Update the agent status to "Not finished"
            self.agents_finished_status[agent_name] = False

        # Pause
        # rospy.sleep(1.0)
        self.exp_start_time = time.time()

        return InitializeResponse(True)

