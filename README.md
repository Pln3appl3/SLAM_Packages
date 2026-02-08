SLAM packages for ros2 humble built from source on Debian Bookworm 64-bit

Create a `slam_packages.repos`
```
repositories:
  external:
    type: git
    url: https://github.com/Pln3appl3/SLAM_Packages.git
    version: main
```

import this repo
```
cd ~/project_name
vcs import src < slam_packages.repos
```
build
```
# build ros2_humble from source https://github.com/Pln3appl3/ros2_humble.git with needed SLAM packages
source ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
