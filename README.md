# CheckPoint 11 Robot Path Planning

<a name="readme-top"></a>

## About The Project
The purpose of this project is to implement a Dijkstra algoritm for obstacle avoidance and path planning.

![This is an image](images/preview.png)

<!-- GETTING STARTED -->
## Getting Started

### Software Prerequisites
* Ubuntu 22.04
* ROS2 Galactic


<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- INSTALLATION -->
### Installation
1. Clone the repo:
   ```sh
   cd ~ && \
   git clone https://github.com/pvela2017/The-Construct-CheckPoint-11-Robot-Path-Planning
   ```
2. Compile the simulation:
   ```sh
   source /opt/ros/galactic/setup.bash && \
   cd ~/The-Construct-CheckPoint-11-Robot-Path-Planning/ros2_ws && \
   colcon build
   ```
     
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE -->
## Usage
### Local Simulation
1. Launch the simulation:
   ```sh
   source /opt/ros/galactic/setup.bash && \
   source ~/The-Construct-CheckPoint-11-Robot-Path-Planning/ros2_ws/install/setup.bash && \
   ros2 launch neo_simulation2 simulation.launch.py
   ```
2. Launch the planner:
   ```sh
   source /opt/ros/galactic/setup.bash && \
   source ~/The-Construct-CheckPoint-11-Robot-Path-Planning/ros2_ws/install/setup.bash && \
   ros2 launch neo_nav2 neo_nav2_full.launch.xml
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- KEYS -->
## Key Topics Learnt
* ROS2 custom planners.