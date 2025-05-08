# visual_loco
Locomotion with visual information

# dependencies
1. yaml-cpp
install yaml-cpp to the system directory:
```
sudo apt update
sudo apt install libyaml-cpp-dev
```

3. ros2 packages
install necessary ros2 packages
```
sudo apt install ros-foxy-joint-state-publisher-gui
```

# Run demos
## Show urdf model of go2 in Rviz
```
source install/setup.bash
ros2 launch go2_description view_model.launch.py
```
