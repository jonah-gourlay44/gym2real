---
layout: default
title: Simulation
parent: Setup
nav_order: 4
---

# Gym2Real_isaacgym
https://github.com/kevinh42/gym2real_isaacgym

This repo provides a custom task to be used with Isaac Gym as well as scripts for training and exporting models.

## Requirements
- Nvidia GPU with 470+ drivers (`nvidia-smi` to make sure these are set up)
- Isaac Gym (tested with version `1.0.preview3`)
- IsaacGymEnvs (tested with commit `9656bac7e59b96382d2c5040b90d2ea5c227d56d`)

### Isaac Gym Setup
- Download Isaac Gym from Nvidia’s developer portal: `https://developer.nvidia.com/isaac-gym` 
- `cd isaacgym/python && pip install -e .` to install Python package
- More detailed installation guide in: `isaacgym/docs/install.html`

### IsaacGymEnvs Setup
- `git clone https://github.com/NVIDIA-Omniverse/IsaacGymEnvs` (tested with commit `9656bac7e59b96382d2c5040b90d2ea5c227d56d`)
- `cd IsaacGymEnvs && pip install -e .` to install Python package

## Repo Structure
- `train.py`: Main script used for training/testing models
- `export.py`: Script for exporting models as ONNX files
- `cfg/task/`: YAML configs for each task defining environment parameters and domain randomization
- `cfg/train/`: YAML configs for each task defining RL model and hyperparameters
- `tasks/`: Class definitions and assets for each task
- `runs/`: Model checkpoints and summaries will be saved here

## IsaacGymEnvs
This repo was designed to be used after installing IsaacGymEnvs (`https://github.com/NVIDIA-Omniverse/IsaacGymEnvs`) as a Python library.
IsaacGymEnvs comes close to being usable as a library, but requires adding your custom task to a map.

The following files are taken from IsaacGymEnvs and modified so that this isn't necessary:
 - `cfg/config.yaml` (minimal changes)
 - `utils/rlgames_utils.py` (minimal changes)
 - `base/vec_task.py` (minimal changes)
 - `train.py`

Instead of looking for tasks in a static map, this code uses pydoc.locate to dynamically load the task class.