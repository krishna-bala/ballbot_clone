# Ballbot Simulation Package

This package depends on the `ballbot_common` package and the `ballbot_simulator` workspace (not included). 

The layout of this package is outlined below.

## Launch File

### `sim_param_sweep.launch`
Launches the nodes `sim_spawn_agents_service_node`, `sim_states_node`, `sim_experiment_node`, and `sim_master_control_node`. 
Also loads the agent configuration file and agent parameters to the `/sim_experiment_node` namespace.

### 

## Nodes

| Node | Descrption | Publishers | Subscribers | Services | Parameters |
| ---  | ---------- | ---------- | ----------- | -------- | ---------- |
|`sim_spawn_agents_service_node`| This node is used to establish the `initialize_agents` service, which deletes all ***sphere*** models from the world and spawns ***spheres*** based on the parameters under the `/sim_experiment_node` namespace.| N/A | N/A | Establishes `initialize_agents`. | N/A |
| `sim_states_node` | This node calls the `initialize_agents` service and begins publishing the states and goals of all agents received from `/gazebo/get_model_state`. | `/agent_name/model_state` (multiple); `/agent_name/goal` (multiple); `/all_agents/model_states`; `/all_agents/published`. | N/A |Requests `/gazebo/get_model_state`. | Retrieves agent goal positions `/sim_experiment_node/agent_name/goal` (multiple) from the parameter server. |
`sim_experiment_node` | Initialize the starting positions for all agents and continuously publish the status of the experiment. The experiment is finished when all agents reach their goal or when the time limit is reached. | `/all_agents/finished` | `/agent_name/model_state` (multiple); `/agent_name/goal` (multiple) | Establishes `initialize_experiment` and requests `initialize_experiment`| Retrieves the start positions of agents from `/sim_experiment_node/agent_name/start` (multiple) |
| `sim_master_control_node` | Serves as the master controller for all agents. Establishes a controller for each agent based on the respective settings established on the parameter server. Every cycle, each controller gets the next desired action based on the FullState and ObservableStates of other agents and logs that action. After all actions are calculated, all actions are executed. | `/cmd_vel` (execute action for ballbot) | `/all_agents/model_states`; `/all_agents/finished`; `/agent_name/goal` (multiple); | `/gazebo/set_model_state` (execute action for spheres) | Retrieves from the `/sim_experiment_node` namespace: `rollout_horizon`; `num_rollouts`; `/span`. |
