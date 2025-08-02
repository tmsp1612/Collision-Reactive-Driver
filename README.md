# Collision-Reactive-Driver
A simple reactive obstacle avoidance algorithm for mobile robots using LiDAR data, implemented in ROS 2 and simulated with Gazebo.


## Instalation
```bash
git clone https://github.com/tmsp1612/Collision-Reactive-Driver
```


## Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## RUN

Before launch the code, first go to launch file and adapt to your robot topic's names.
Then adjust on the BumpGoNode.hpp file, located in include folder, the values for rotation and speed time for your robot
After that just run:
```bash
ros2 launch bumpandgo bump_and_go.launch.py 
```

## Expriments
The follow repo was tested with the robot on this [link]([https://github.com/tmsp1612/Velodyne_Simulator_Humble](https://github.com/tmsp1612/BCR_BOT_LIDAR))




