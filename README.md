Provides a Jackal Gym simulated environment using the Gazebo simulator. In order
to run it please following the following steps:

# Create a ROS catkin workspace
```console
mkdir -p ~/catkin_ws/src
```
# Clone the repository
```console
cd ~/catkin_ws/src
git clone git@github.com:ICGog/jackal-gym.git
```

# Create a Docker image
```console
cd jackal-gym/docker
./build.sh jackal-gym
```
# Run the container
```console
./run.sh jackal-gym
source /opt/ros/indigo/setup.bash
```

# Build the project
```console
cd ~/catkin_ws/
catkin_make
catkin_make install
```

# Run the Jackal Gazebo simulation
In order to start Jackal with a LiDAR please run:
```console
roslaunch jackal-gym jackal_world.launch config:=front_laser
```

Alternatively, you can run the following command to equip Jackal with a front camera:
```console
roslaunch jackal-gym jackal_world.launch config:=front_bumblebee2
```

# Running Q-learning training
Run in a different terminal:
```console
./connect_to_container.sh <container_id>
roslaunch jackal-gym jackal_rl.launch

```

# Launching rviz
If you want to visualize camera and LiDAR feeds you can start rviz by executing the following commands:
```console
./connect_to_container.sh <container_id>
roslaunch jackal_viz view_robot.launch
```
