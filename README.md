# Ballbot ROS Integration
This workspace hosts ROS controller packages that integrate with the `ballbot_simulator` workspace (not included) in Gazebo and on the physical platform.

To use the ROS packages, clone this repo and run `catkin_make` in the base directory.

The four packages in this workspace are outlined below.
## `ballbot_common`
This package implements controllers for all agents and the data recording utility. The following controllers can be used for agents in both real and simulation:
- PID
- ORCA
- CADRL
- MPC

See the `ballbot_common` ***README*** for additional information.

## `ballbot_sim`
This package contains the ROS interface for the Gazebo simulator. 
The agents consist of the ballbot (ego agent) and spheres (other agents). 
All models are defined in the `ballbot_simulator` package (not included in this repository).


See the `ballbot_sim` ***README*** for additional information.

## `ballbot_real`
This package contains the ROS interface for the physical platform and agents (humans) tracked by MoCap.

See the `ballbot_real` ***README*** for additional information.

## `mocap`
This package implements the `topic_tools/relay` node to be used with the `ballbot_real` package.