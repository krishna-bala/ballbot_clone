# Ballbot Real Package

This package depends on the `ballbot_common` package and the `ballbot_simulator` workspace (not included). 

Work in progress. Operates similarly to the `ballbot_sim` package -- the main difference is the subscribed topics.


## Nodes

| Node | Descrption | Publishers | Subscribers | Services | Parameters |
| ---  | ---------- | ---------- | ----------- | -------- | ---------- |
| `real_states_node` | This node publishes the states to and goals of all agents. States are received from `/agent_name/pose` and published to `/agent_name/model_state`. Goals are received from the parameter server and published to `/agent_name/goal`. | `/agent_name/model_state` (multiple); `/agent_name/goal` (multiple); `/all_agents/model_states`. | `/agent_name/pose` | | Retrieves agent goal positions `/real_experiment_node/agent_name/goal` (multiple) from the parameter server. |
| `real_master_control_node` | Serves as the master controller for ballbot. Establishes a controller for ballbot on the respective settings established on the parameter server. Every cycle, the controller gets the next desired action based on the FullState of ballbot and ObservableStates of other agents and executes the action. | `/cmd_vel` (execute action for ballbot) | `/all_agents/model_states`; `/all_agents/finished`; `/ballbot/goal` (multiple); | | Retrieves from the `/real_experiment_node` namespace: `rollout_horizon`; `num_rollouts`; `/span`. |
