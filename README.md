# **Collision-Avoidance-System-for-Autonomous-Robot-Navigation-in-Indoor-Environments**

This repository focuses on the development of a **collision avoidance system for a robot operating in a two-dimensional environment**. The algorithm enables the robot to navigate within a building while autonomously avoiding obstacles, ensuring smooth and uninterrupted movement.

The implementation is based on **ROS (Robot Operating System)** and utilizes laser scanners to detect obstacles. The collected data is transformed into spatial coordinates, allowing the robot to process and execute appropriate responses. The algorithm is inspired by the **Artificial Potential Field method**, calculating repulsive forces to steer the robot away from obstacles and attractive forces to maintain the desired direction.

The system employs ROS topics to manage communication between different components: movement commands are first sent to an auxiliary topic, processed, and then forwarded to the robot with necessary corrections to prevent collisions. The result is an optimized movement strategy that enables the robot to navigate smoothly, adjusting its linear and angular velocities according to the distance from surrounding objects.

# **Execution**

Here are the steps to run the project:

1- **Clone the labiagi_2020_21 repository**
   
   ``` bash
   git clone https://gitlab.com/grisetti/labiagi_2020_21.git 
   ```
