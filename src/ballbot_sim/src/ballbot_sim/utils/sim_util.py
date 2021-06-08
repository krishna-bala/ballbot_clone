import rospy
from gazebo_msgs.srv import GetWorldProperties


def load_cost_params(agent_name):
    sigma = {'h': rospy.get_param('/sim_experiment_node/' + agent_name + '/cost_params/sigma/h'),
             's': rospy.get_param('/sim_experiment_node/' + agent_name + '/cost_params/sigma/s'),
             'r': rospy.get_param('/sim_experiment_node/' + agent_name + '/cost_params/sigma/r')}

    q = {'obs': rospy.get_param('/sim_experiment_node/' + agent_name + '/cost_params/q/obs'),
         'goal': rospy.get_param('/sim_experiment_node/' + agent_name + '/cost_params/q/goal'),
         'wind': rospy.get_param('/sim_experiment_node/' + agent_name + '/cost_params/q/wind')}

    params = {'sigma': sigma, 'q': q}
    return params


def get_names():
    """
    Gets world properties from Gazebo and extract model names from each property.

    Returns:
        list: list of names (string)
    """
    names = []
    rospy.wait_for_service('/gazebo/get_world_properties')
    models_props = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    models = models_props()
    for name in models.model_names:
        if 'sphere' in name or 'ballbot' in name:
            names.append(name)
    return names
