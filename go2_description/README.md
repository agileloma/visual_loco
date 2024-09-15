## go2_description
This repository contains the urdf model of the Go2 robot.

## Build the library
Create a new colcon workspace:
```
# Create the directories
mkdir -p <loco_ws>/src
cd <loco_ws>/src
git clone git@github.com:agileloma/visual_loco.git

# build the workspace
cd <loco_ws>
colcon build
```

## Run the library
```
# Show urdf model of go2 in Rviz
source install/setup.bash
ros2 launch go2_description view_model.launch.py
```