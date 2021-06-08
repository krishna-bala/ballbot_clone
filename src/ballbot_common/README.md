# Ballbot Controllers Package

This package contains the controller implementations for both `ballbot_sim` and `ballbot_real`.

## PID
Simple PID controller for testing purposes. Not used for evaluation.

## ORCA
PyRVO2 implementation of ORCA. See https://github.com/sybrenstuvel/Python-RVO2. 

## CADRL
ROS implmentation of the CADRL policy. See https://github.com/mit-acl/rl_collision_avoidance. Checkpoints for this policy were trained using ballbot dynamics and a modified reward structure. Specifically, the policy was initialized with the IROS18 GA3C-CADRL policy and trained with the updated dynamics and reward structure.

## MPC
Model Predictive Controller that uses *distance-to-goal*, *distance-to-obstacles*, and *winding-number* as heuristics for the cost function. 
Rollouts for the MPC are generated using `TrajGen`, which allows us to modify the control policy used to generate each rollout. 
Currently, `TrajGen` implements `CADRL` as the rollout control policy.
Rollouts are evaluated using `TrajEval`.
