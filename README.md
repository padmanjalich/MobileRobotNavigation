# Racecar Simulation with Pybullet

## Project Description

This project utilizes the Pybullet library to simulate a racecar equipped with a Hokuyo lidar sensor in a dynamic environment. The simulation aims to develop and test intelligent robot programming and navigation algorithms using real-time sensor data and inverse kinematics.

## Objectives

- Simulate a racecar with lidar sensor capabilities.
- Implement intelligent control and navigation algorithms.
- Navigate through a dynamic environment with obstacles.

## Project Workflow

1. **Setup:**
   - Install Pybullet and set up a Python environment.
   - Load the project into your preferred IDE.

2. **Simulation Environment:**
   - The environment includes a racecar, a flat plane, and twelve randomly positioned marble cubes as dynamic obstacles.

3. **Lidar Sensor:**
   - The racecar uses a Hokuyo lidar sensor to perceive its surroundings.
   - Ray-casting is employed to detect collisions and gather information about obstacle locations.

4. **Control Mechanism:**
   - The steering control mechanism adjusts the racecar's trajectory based on lidar readings.
   - A steering algorithm analyzes sensor data to determine the optimal path.

5. **Tasks:**
   - Load obstacles with specific characteristics using URDF files.
   - Implement robot control and navigation functions.
   - Apply forward and inverse kinematics for precise movements.

## Key Learnings

- Gain hands-on experience in simulating and programming mobile robots.
- Develop navigation algorithms for autonomous robot control.
- Understand the challenges of intelligent robotic navigation in dynamic environments.
