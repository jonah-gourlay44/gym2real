---
layout: default
title: Model Training
parent: Usage
nav_order: 1
---

# Gym2Real_isaacgym
https://github.com/kevinh42/gym2real_isaacgym

This repo provides a custom task to be used with Isaac Gym as well as scripts for training and exporting models.

## Repo Structure
- `train.py`: Main script used for training/testing models
- `export.py`: Script for exporting models as ONNX files
- `cfg/task/`: YAML configs for each task defining environment parameters and domain randomization
- `cfg/train/`: YAML configs for each task defining RL model and hyperparameters
- `tasks/`: Class definitions and assets for each task
- `runs/`: Model checkpoints and summaries will be saved here

## Usage
### Adding a New Task
Create a new folder called `tasks/{task_name}`. The following files will be needed:
- `tasks/{task_name}/{task_name}.py`: Class defining the task. Rewards, observations, actions are all defined here.
- `tasks/{task_name}/assets/`: URDF and meshes should be placed here.
- `cfg/task/{TaskName}.yaml`: Config file defining environment parameters and domain randomization
- `cfg/train/{TaskName}PPO.yaml`: Config gile defining hyperparameters for training

### Training
- `python train.py task={task_name}` to begin training
- Models are saved as `runs/{TaskName}/nn/{checkpoint_name}.pth`

### Exporting ONNX
- `python export.py task={task_name} checkpoint="runs/{TaskName}/nn/{checkpoint_name}.pth"` to export
- Model is exported to `runs/{TaskName}/nn/{checkpoint_name}.pth.onnx`

## `train.py`

### `__init__`
Loads configured parameters and sets up tensors.

### `_create_envs`
Loads assets and creates agents. DOFs (joints) and rigid bodies (links) are configured here.

### `compute_reward`
Calculates reward from observations.

### `compute_observations`
Data from environments are placed in the observations buffer for calculating reward and updating neural network.

### `reset_idx`
Anything that should be applied to the agent on reset can be added here (eg. randomization, initial pose, initial DOF states)

### `pre_physics_step`
This step occurs after the network has been updated and has outputted an action. This action can be applied to control the agent here (eg. position/velocity/effort control of DOFs). Physics simulation happens after this step.

### `post_physics_step`
This step occurs after any physics simulation has finished. Observations and rewards are computed here for the next step.