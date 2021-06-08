import rospy
import rospkg
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel

from ballbot_sim.utils.sim_util import get_names


# noinspection PyAttributeOutsideInit
class SimSpawnAgents(object):

    def __init__(self):

        rospack = rospkg.RosPack()
        pack_path = rospack.get_path('ballbot_sim')
        self.ballbot_sim_model_path = pack_path + '/../../../ballbot_simulator/src/ballbot_env/models/'
        self.setup_srv()

    def setup_srv(self):
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.wait_for_service("gazebo/delete_model")
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    def delete_existing_agents(self):
        existing_agents = get_names()
        for agent in ['sphere1', 'sphere2', 'sphere3', 'sphere4', 'sphere5']:
            if agent in existing_agents:
                self.delete_model(agent)

    def spawn_agents(self):
        param_names = rospy.get_param_names()
        node_ns = '/sim_experiment_node'
        for i, agent in enumerate(['sphere1', 'sphere2', 'sphere3', 'sphere4', 'sphere5']):
            agent_param = node_ns + '/' + agent + '/start'
            if agent_param in param_names:
                with open(self.ballbot_sim_model_path + agent + '/model.sdf', "r") as f:
                    agent_xml = f.read()
                agent_pose = Pose()
                # Arbitrary placement, make sure no collisions with ballbot (default ballbot pos is (0,0) )
                agent_pose.position.x = 10 + 2 * i
                agent_pose.position.y = 10 + 2 * i
                self.spawn_model(agent, agent_xml, "/", agent_pose, "world")

