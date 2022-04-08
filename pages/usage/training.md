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