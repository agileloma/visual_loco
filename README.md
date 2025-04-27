# visual_loco
Locomotion with visual information

# dependencies
1. unitree_sdk2
install unitree_sdk2 to a specified directory:

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree
sudo make install
```
Note that if you install the library to other places other than `/opt/unitree`, 
you need to make sure the path is added to "${CMAKE_PREFIX_PATH}" so that cmake 
can find it with "find_package()".

2. yaml-cpp
install yaml-cpp to the system directory:
```
sudo apt update
sudo apt install libyaml-cpp-dev
```

