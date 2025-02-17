# **Collision-Avoidance-System-for-Autonomous-Robot-Navigation-in-Indoor-Environments**

This repository focuses on the development of a **collision avoidance system for a robot operating in a two-dimensional environment**. The algorithm enables the robot to navigate within a building while autonomously avoiding obstacles, ensuring smooth and uninterrupted movement.

The implementation is based on **ROS (Robot Operating System)** and utilizes laser scanners to detect obstacles. The collected data is transformed into spatial coordinates, allowing the robot to process and execute appropriate responses. The algorithm is inspired by the **Artificial Potential Field method**, calculating repulsive forces to steer the robot away from obstacles and attractive forces to maintain the desired direction.

The system employs ROS topics to manage communication between different components: movement commands are first sent to an auxiliary topic, processed, and then forwarded to the robot with necessary corrections to prevent collisions. The result is an optimized movement strategy that enables the robot to navigate smoothly, adjusting its linear and angular velocities according to the distance from surrounding objects.

# **Execution**

At first, we have to start the environment: 

1- **Clone the labiagi_2020_21 repository**
   
   ``` bash
   git clone https://gitlab.com/grisetti/labiagi_2020_21.git 
   ```

2- **Execute the web control to start the map and rviz through the following code lines**

  ``` bash
  cd ~/labiagi_2020_21/workspaces/srrg2_labiagi/src/srrg2_navigation_2d/config
  source /labiagi_2020_21/srrg2_webctl/proc_webctl run_navigation.webctl
  ```

3- **Open a web browser and go to the  http://localhost:9001/ address**

4- **Press the first five buttons in the table displayed on the webpage**

Them, we can run the collision avoidance algorithm:

1- **Open a new terminal**, **navigate to the project folder** and **execute *catkin_make***

2- **Source the setup file**

   ``` bash
   source devel/setup.bash
   ```

3- **Run the Collision Avoidance program**

   ``` bash
   rosrun collision_avoidance CollisionMain
   ```

4- **Open another terminal and run the following command to send velocity commands**

   ``` bash
   rostopic pub /cmd_vel_AUX /geometry_msgs/Twist -r 1
   ```
   
   
  
