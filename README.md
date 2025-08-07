# Safeguarding Autonomous Collaborative Robots Using Ambient Sensors

## Overview
In industrial settings where humans and robots share the same workspace, safety is often enforced through physical barriers or complete shutdowns—methods that sacrifice productivity. This project proposes a dynamic, sensor-driven safety system for collaborative robots (cobots), leveraging real-time human detection and intelligent control to halt robot motion proactively when a human enters the workspace.

## Technologies Used

- **Robot**: UR16e Collaborative Robotic Arm
- **Framework**: ROS 2 (Humble), Ubuntu 22.04
- **Detection**: YOLOv7-tiny (Pre-trained)
- **Camera**: Arducam IMX477 (12MP, 4K @ 30FPS)
- **Compute Unit**: NVIDIA Jetson Orin Nano
- **Visualization**: RViz
- **Programming**: Python, C++

## Problem Statement
Traditional safety methods slow down production by forcing complete stops when a human enters a shared space. We aimed to build a more intelligent and efficient system that detects humans in real time and halts the robot only when necessary, minimizing disruption.

## Methodology

1. **Set up the UR16e arm and camera using ROS 2.**
2. **Integrated YOLOv7-tiny on the Jetson Orin Nano for high-speed, real-time human detection.**
3. **Wrote ROS 2 nodes to publish trajectories and issue zero-velocity commands upon detection.**
4. **Maintained continuous monitoring of the workspace and introduced a pause-resume safety logic via trajectory control.**

## Experimental Setup

- **The UR16e was connected to the Jetson via Ethernet.**
- **Arducam IMX477 streamed image frames directly to the Orin Nano.**
- **Human detection triggered an emergency stop with a 10-second pause.**
- **Robot resumed movement after confirming the workspace was clear.**
- **All interactions were visualized in RViz for debugging and verification.**

## Key Results

- **Collision response time was reduced by ~90% compared to traditional safety systems.** This was measured by benchmarking our system’s stop latency (~50–70 ms) against industry averages (~500–700 ms).
- **Safety compliance was 95%,** confirmed through 20 test trials where 19 resulted in successful detection and halting.
- **Real-time operation was consistent under varied lighting and workspace conditions.**

## Future Scope

- **Introduce LiDAR and stereo cameras** for better depth perception and dynamic zone monitoring.
- **Add adaptive path planning** to avoid halting and instead reroute around humans.
- **Deploy cloud-based logging and analytics** for long-term performance tracking and model updates.

