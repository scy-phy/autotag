# Setup

In this file, we describe how to setup the simulation environment. 

Our testbed is based on the **TUM Carla–Autoware Bridge** together with **Autoware Universe 2024.01** and **Carla v0.9.15**. We build upon the original bridge implementation but include several modifications required for our work.

The setup is based on the official TUM Carla–Autoware Bridge:

- https://github.com/TUMFTM/Carla-Autoware-Bridge

The bridge included in this repository is **not** the original upstream bridge. Instead, it is a Git subtree of the TUM Carla–Autoware Bridge containing additional modifications required for our work.

To reproduce our setup, first follow the installation instructions provided by the original bridge repository, but apply the modifications described below.

## Differences from the Original Setup

The following changes are required compared to the original setup:

- Use **Autoware Universe 2024.01** instead of the version referenced in the original documentation:
  `docker pull ghcr.io/autowarefoundation/autoware:humble-2024.01-cuda-amd64`
- Apply the ROS key fix described in https://github.com/TUMFTM/Carla-Autoware-Bridge/issues/56 before building the Autoware workspace.
- Use the modified bridge contained in this repository (`/bridge`) instead of cloning the upstream bridge.
- Apply the additional Autoware modifications contained in the `/autoware` directory. These changes are required to enable the multi-sensor fusion pipeline of Autoware. 
- The bridge Docker container must be started with additional volume mounts to include the modified bridge code. The exact commands are documented in `run.md`.

## Prerequisites

Before running the setup, ensure that:

- Ubuntu 20.04 or 22.04 is used.
- An NVIDIA GPU is available for running CARLA.
- Docker is installed and configured with GPU support (e.g., via the NVIDIA Container Toolkit).
- Rocker is installed.
- A `cyclonedds.xml` configuration file is available to configure CycloneDDS. 