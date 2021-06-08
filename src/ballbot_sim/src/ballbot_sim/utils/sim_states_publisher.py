import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import GetModelState


# noinspection PyAttributeOutsideInit
class SimStatesPublisher(object):
    """
    Using the agent names provided, this class retreives the model states from gazebo (service) and publishes the
    states to the appropriate topics.
    """

    def __init__(self, agent_names):
        self.names = agent_names

        # Dictionary of agent goals as they are initialized from the parameter server
        self.agent_goals = {}
        self.sim_agent_goal_pubs = {}
        self.sim_agent_model_state_pubs = {}
        self.setup_srv()
        self.setup_pubs()

    def setup_srv(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def setup_pubs(self):
        """
        Gets goal for each agent from param server and publishes to the appropriate goal topic.
        Gets model state for each agent from /gazebo/get_model_state service and publishes to the appropriate model
        state topic.

        Publishers:
            /agent_name/goal (multiple)
            /agent_name/model_state (multiple)
        """

        for agent_name in self.names:
            self.sim_agent_model_state_pubs[agent_name] = rospy.Publisher('/' + agent_name + '/model_state', ModelState,
                                                                          queue_size=10)
            self.sim_agent_goal_pubs[agent_name] = rospy.Publisher('/' + agent_name + '/goal', Point, queue_size=10)
            try:
                goal = rospy.get_param('/sim_experiment_node/' + agent_name + '/goal')
            except KeyError:
                print(
                            "You need to set the ROS Parameter Server first for /sim_experiment_node/" + agent_name + "/goal\n")
                return
            # initialize our goal dict with each agent's singular goal
            self.agent_goals[agent_name] = goal

        self.all_agents_pub = rospy.Publisher('/all_agents/model_states', ModelStates, queue_size=10)

    def publish_states(self, ref_frame='world'):
        """
        Gets the ModelState for each agent in Gazebo and publishes to /agent_name/model_state.
        Also publishes all agent ModelStates to /all_agents/model_states

        :param ref_frame: the default reference frame is world. No need to modify.
        """

        name_list = []
        pose_list = []
        twist_list = []

        for agent_name in self.sim_agent_model_state_pubs:
            agent_model_state_pub = self.sim_agent_model_state_pubs[agent_name]

            state = self.get_model_state(agent_name, ref_frame)
            agent = ModelState()
            agent.model_name = agent_name
            agent.pose = state.pose
            agent.twist = state.twist
            agent.reference_frame = ref_frame

            name_list.append(agent.model_name)
            pose_list.append(agent.pose)
            twist_list.append(agent.twist)

            agent_model_state_pub.publish(agent)

        all_agents = ModelStates()
        all_agents.name = name_list
        all_agents.pose = pose_list
        all_agents.twist = twist_list
        self.all_agents_pub.publish(all_agents)

    def publish_goals(self):
        """
        Publishes the goals assigned at initializtion to /agent_name/goal. Does not publish goals that have been
        modified since initialization.
        """
        for agent_name in self.sim_agent_goal_pubs:
            agent_goal_pub = self.sim_agent_goal_pubs[agent_name]
            gx, gy = self.agent_goals[agent_name]
            goal = Point()
            goal.x = gx
            goal.y = gy
            agent_goal_pub.publish(goal)
