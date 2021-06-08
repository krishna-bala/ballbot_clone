import rospy
from geometry_msgs.msg import PoseStamped, Point
from gazebo_msgs.msg import ModelStates, ModelState


class RealStatesSubPub(object):

    def __init__(self, agent_names):
        self.agent_names = agent_names

        self.real_agent_pose_stamped_subs = {}
        self.all_agents_prev_pose_stamped = {}
        self.all_agents_model_state = {}
        self.agent_goals = {}

        self.real_agent_model_state_pubs = {}
        self.real_agent_goal_pubs = {}
        self.all_agents_pub = None

        self.setup_subs()
        self.setup_pubs()

    def setup_subs(self):

        for agent_name in self.agent_names:
            self.real_agent_pose_stamped_subs[agent_name] = rospy.Subscriber('/' + agent_name + '/pose', PoseStamped,
                                                                             self.get_agents, callback_args=agent_name)
            try:
                goal = rospy.get_param('/real_experiment_node/' + agent_name + '/goal')
            except KeyError:
                print("You need to set the ROS Parameter Server first for /real_experiment_node/" + agent_name +
                      "/goal\n")
                return
            self.agent_goals[agent_name] = goal

    def get_agents(self, msg, agent_name):

        # First time through callback, create ModelState in all agents dict
        if agent_name not in self.all_agents_model_state:
            self.all_agents_model_state[agent_name] = ModelState()

        curr_time = msg.header.stamp.secs + (msg.header.stamp.nsecs * 1e-9)
        # If it's not the first callback
        if agent_name in self.all_agents_prev_pose_stamped:
            prev_pose_stamped = self.all_agents_prev_pose_stamped[agent_name]
            dx = msg.pose.position.x - prev_pose_stamped.pose.position.x
            dy = msg.pose.position.y - prev_pose_stamped.pose.position.y
            prev_time = prev_pose_stamped.header.stamp.secs + (prev_pose_stamped.header.stamp.nsecs * 1e-9)
            dt = curr_time - prev_time
            self.all_agents_model_state[agent_name].twist.linear.x = dx / dt
            self.all_agents_model_state[agent_name].twist.linear.y = dy / dt

        self.all_agents_prev_pose_stamped[agent_name] = PoseStamped()
        self.all_agents_prev_pose_stamped[agent_name].header = msg.header
        self.all_agents_prev_pose_stamped[agent_name].pose = msg.pose

        self.all_agents_model_state[agent_name].pose = msg.pose

    def setup_pubs(self):
        for agent_name in self.agent_names:
            self.real_agent_model_state_pubs[agent_name] = rospy.Publisher('/' + agent_name + '/model_state',
                                                                           ModelState, queue_size=10)
            self.real_agent_goal_pubs[agent_name] = rospy.Publisher('/' + agent_name + '/goal', Point, queue_size=10)
        self.all_agents_pub = rospy.Publisher('/all_agents/model_states', ModelStates, queue_size=10)

    def publish_states(self):
        name_list = []
        pose_list = []
        twist_list = []

        for agent_name in self.agent_names:
            agent = self.all_agents_model_state[agent_name]

            name_list.append(agent.model_name)
            pose_list.append(agent.pose)
            twist_list.append(agent.twist)
            self.real_agent_model_state_pubs[agent_name].publish(agent)

        all_agents = ModelStates()
        all_agents.name = name_list
        all_agents.pose = pose_list
        all_agents.twist = twist_list
        self.all_agents_pub.publish(all_agents)

    def publish_goals(self):
        """
        Publishes the goals assigned at initializtion to /agent_name/goal. Does not publish goals that have been modified since
        initialization.
        """
        for agent_name in self.agent_goals:
            gx, gy = self.agent_goals[agent_name]
            goal = Point()
            goal.x = gx
            goal.y = gy
            self.real_agent_goal_pubs[agent_name].publish(goal)
