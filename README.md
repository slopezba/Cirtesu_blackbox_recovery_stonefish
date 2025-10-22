# 🐠 BlueROV2 Stonefish Simulator

![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)
![Python 3](https://img.shields.io/badge/Python-3.8+-3776AB?logo=python&logoColor=white)
![ArduSub](https://img.shields.io/badge/ArduSub-SITL-blue)
![Stonefish Simulator](https://img.shields.io/badge/Simulator-Stonefish-00BFFF)

---

A **BlueROV2 underwater vehicle simulator** using the **Stonefish** environment and **ROS Noetic**.  
This project allows you to run a realistic ROV simulation, control it with a **joystick**, and link it with **ArduSub** as if it were a real robot.

---

## 🧩 Prerequisites

Before starting, make sure you have the following installed:

- **Ubuntu 20.04**
- **ROS Noetic**

## ⚙️ Initial Setup

Load the ROS Noetic environment:

`source /opt/ros/noetic/setup.bash`

Build the workspace:

`catkin build`

Load the workspace environment:

`source devel/setup.bash`

## 🧪 Launch the Stonefish Simulator

Run the BlueROV2 simulation in a pipeline (pipe) environment:

`roslaunch bluerov_simulation_blackbox bluerov_pipe.launch`

This will start the Stonefish visual simulator with the BlueROV2 model and automatically open RViz for visualization.

## 🔗 Link with ArduSub (Control as a Real Robot)

In another terminal, start ArduSub with the JSON model configuration and the correct communication ports:

`sim_vehicle.py -v ArduSub --model JSON --map -L PHILL \
  --out=127.0.0.1:14555 \
  --out=127.0.0.1:9003`

Then run the bridge patch to link the simulator with ArduSub:

`rosrun simulation ardusim_patch.py`

## 🎮 Control with Joystick

To manually control the vehicle using a joystick:

Launch the movement control nodes:

`roslaunch bluerov_simulation_blackbox movement.launch`

Run the required simulation servers:

`rosrun bluerov_simulation_blackbox server_sim.py`
`rosrun bluerov_simulation_blackbox server_sim.py`





