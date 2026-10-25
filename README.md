# edge-cloud-poc

Low-latency cloud-edge robot control with ROS 2, Gazebo and Nebius Token Factory (NVIDIA Nemotron).

**Demo task:** sort 3 colored blocks into bins by color from a natural-language command.

- Edge layer: ROS 2 + Gazebo robot arm (Docker)
- Cloud layer: planner service calling NVIDIA-Nemotron-3-Nano-30B-A3B on Nebius Token Factory (Docker)
- Goal: measure and reduce end-to-end latency between the edge and the cloud
