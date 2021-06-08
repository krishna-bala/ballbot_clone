import rospy
from gazebo_msgs.srv import GetWorldProperties


def get_real_names(num_obs=0):
    names = {'pathbot'}
    for i in range(num_obs):
        names.add('agent' + str(i + 1))
    return names


def load_cost_params(agent_name):
    sigma = {'h': rospy.get_param('/real_experiment_node/' + agent_name + '/cost_params/sigma/h'),
             's': rospy.get_param('/real_experiment_node/' + agent_name + '/cost_params/sigma/s'),
             'r': rospy.get_param('/real_experiment_node/' + agent_name + '/cost_params/sigma/r')}

    q = {'obs': rospy.get_param('/real_experiment_node/' + agent_name + '/cost_params/q/obs'),
         'goal': rospy.get_param('/real_experiment_node/' + agent_name + '/cost_params/q/goal'),
         'wind': rospy.get_param('/real_experiment_node/' + agent_name + '/cost_params/q/wind')}

    params = {'sigma': sigma, 'q': q}
    return params
