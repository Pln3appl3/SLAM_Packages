
# Documentation:

## 2/1/26
I Setup all the files, package.xml, CMakeLists.txt, and .msg and .srv for the project and already prebuilt ros2 Hubmble from soruce on debian bookworm (64-bit), but I'm having problems getting the dependencies for ros2 packages and getting them to build correctly.

created `tmr.repos` 
```
repositories:
  # Core Navigation
  external/navigation2:
    type: git
    url: https://github.com/ros-planning/navigation2.git
    version: humble
  
  # SLAM
  external/rtabmap_ros:
    type: git
    url: https://github.com/introlab/rtabmap_ros.git
    version: humble-devel
  external/rtabmap:
    type: git
    url: https://github.com/introlab/rtabmap.git
    version: master
  
  # Vision/Perception deps
  external/vision_opencv:
    type: git
    url: https://github.com/ros-perception/vision_opencv.git
    version: humble
  external/image_common:
    type: git
    url: https://github.com/ros-perception/image_common.git
    version: humble
  external/image_pipeline:
    type: git
    url: https://github.com/ros-perception/image_pipeline.git
    version: humble
  
  # Utils
  external/angles:
    type: git
    url: https://github.com/ros/angles.git
    version: humble-devel
  external/bond_core:
    type: git
    url: https://github.com/ros/bond_core.git
    version: humble
  external/diagnostics:
    type: git
    url: https://github.com/ros/diagnostics.git
    version: humble
  external/behaviortree_cpp_v3:
    type: git
    url: https://github.com/BehaviorTree/BehaviorTree.CPP.git
    version: v3.8  # Nav2 Humble uses v3
  
  # EKF
  external/robot_localization:
    type: git
    url: https://github.com/cra-ros-pkg/robot_localization.git
    version: humble-devel
  
  # Teleop
  external/teleop_twist_keyboard:
    type: git
    url: https://github.com/ros2/teleop_twist_keyboard.git
    version: humble
```
import them
```
cd ~/tmr_ws
vcs import src < tmr.repos
```

SO I decided to check and install dependencies from the start,
```
sudo pip3 install -U colcon-common-extensions vcstool rosdep --break-system-packages
```

```
sudo rosdep init || true
rosdep update
```

```
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv \
    python3-empy \
    python3-numpy \
    python3-yaml \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libtinyxml2-dev \
    libtinyxml-dev \
    libasio-dev \
    libpoco-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    libzstd-dev \
    liblz4-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libprotobuf-dev protobuf-compiler \
    libqt5gui5 libqt5widgets5 \
    libxaw7-dev \
    libspdlog-dev \
    libceres-dev \
    libsqlite3-dev \
    libusb-1.0-0-dev \
    libudev-dev
```
```
rosdep install --from-paths src --ignore-src -r -y
```

Built rtabmap from source
```
cd src/external/rtabmap/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

Tried building again but rtabmap_msgs failed again and says it couldn't find rcutils which i also couldn't find in my ros2_humble so i decided to rebuild it because it may have originally been built with `colcon build --merge-install` which led rcutils not being built.
```
cd ~/ros2_humble
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
The rebuilding failed and gave an error at mcap_vendor but since i dont need it i decided to skip mcap and rerun.
```
cd ~/ros2_humble
rm -rf build install log
colcon build --symlink-install --packages-skip mcap_vendor rosbag2_storage_mcap rosbag2_storage_mcap_testdata
```
didnt work because of mcap so i disabled -Werror in MCAP by adding `add_compile_options(-Wno-error)` to the top of `rosbag2/mcap_vendor/CMakeLists.txt` then rebuilt 
```
cd ~/ros2_humble
rm -rf build install log
colcon build --symlink-install
```
then i tried rebuilding my project but i still got errors on rcutils and found a error in the rtabmap_msgs CMakeLists.txt that i needed to fix so I removed this from rtabmap_msgs CMakeLists.txt
```
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    rcutils_LIB NAMES rcutils
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()
```
because rtabmap_msgs does not need to manually find rcutils. ROS 2 handles that automatically through ament.
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```
and now it doesn't give an error at rtabmap_msgs and builds it properly.

Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_dwb_controller/dwb_critics/CMakeLists.txt` under `project()` due to GCC 12 like before. Then rebuilt
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```
Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_dwb_controller/nav_2d_utils/CMakeLists.txt` under `nav2_package()` and above `ament_package()` to make sure it doesnt get overridden. Then built nav_2d_utils alone
```
source ~/ros2_humble/install/setup.bash
colcon build --packages-select nav_2d_utils --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```
Build project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_dwb_controller/dwb_plugins/CMakeLists.txt` under `nav2_package()` and above `ament_package()` to make sure it doesnt get overridden. Then built project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_dwb_controller/dwb_critics/CMakeLists.txt` under `nav2_package()` and above `ament_package()` to make sure it doesnt get overridden. Then built project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

removed
```
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()
```
from `~/tmr_ws/src/external/rtabmap_ros/rtabmap_conversions/CMakeLists.txt` just like i did for rtabmap_msgs and for the same reasons. Then i rebuilt
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_behaviors/CMakeLists.txt` under `nav2_package()` and above `ament_export_dependencies(${dependencies})` to make sure it doesnt get overridden. Then built project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_bt_navigator/CMakeLists.txt` under `nav2_package()` and above `ament_package()` to make sure it doesnt get overridden. Then built project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_constrained_smoother/CMakeLists.txt` under `nav2_package()` and above `ament_package()` to make sure it doesnt get overridden. Then built project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_mppi_controller/CMakeLists.txt` under `nav2_package()` and above `ament_package()` to make sure it doesnt get overridden. Then built project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

Added `add_compile_options(-Wno-error)` to `~/tmr_ws/src/external/navigation2/nav2_navfn_planner/CMakeLists.txt` under `nav2_package()` and above `ament_package()` to make sure it doesnt get overridden. Then built project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

decided to Add `add_compile_options(-Wno-error)` to all the nav2 CMakeLists.txt under `nav2_package()` and above `ament_package()` to make sure it doesnt get overridden. Then built project again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

Got an error with image_publisher, i need to figure wht that happened tomorrow and how to fix it.

## 2/2/26
switched rtabmap to the correct branch and rebuild
```
cd ~/tmr_ws/src/external/rtabmap
git fetch
git checkout humble-devel
```
```
cd ~/tmr_ws
rm -rf build install log
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```
changed rtabmap version in `tmr.repos` from `master` to `humble-devel`because rtabmap was trying to use ros1 PCL conversion APIs but i am using ROS2 Humble PCL conversion APIs
removed `-Werror` from `add_compile_options()` in `nav2_rviz_plugins/CMakeLists.txt` and rebuilt
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```
and this code seems to not help

changed `param_t value;` to `param_t value = default_value;` in `~/tmr_ws/src/external/navigation2/nav2_dwb_controller/nav_2d_utils/include/nav_2d_utils/parameters.hpp` since head cannot compile under GCC 12 as param_t value is uninitialized.
```
template<class param_t>
param_t searchAndGetParam(
  const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & param_name,
  const param_t & default_value)
{
  // Initialize value so GCC12 cannot complain
  param_t value = default_value;

  nav2_util::declare_parameter_if_not_declared(
    nh, param_name,
    rclcpp::ParameterValue(default_value));

  nh->get_parameter(param_name, value);
  return value;
}
```
rebuild nav_2d_utils alone with its dependencies
```
rm -rf ~/tmr_ws/build/nav_2d_utils ~/tmr_ws/install/nav_2d_utils
rm -rf ~/tmr_ws/build/dwb_critics ~/tmr_ws/install/dwb_critics
rm -rf ~/tmr_ws/build/dwb_plugins ~/tmr_ws/install/dwb_plugins
```
```
source ~/ros2_humble/install/setup.bash
colcon build --packages-select nav_2d_utils dwb_critics dwb_plugins --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```
all three packages finished fine and now i will build whole workspace
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

rtabmap_conversions fails because it is not taking humble-devel version of rtabmap so i decided to vcs only those repos again and build.

created rtabmap git only .repos
```
repositories:
  external/rtabmap_ros:
    type: git
    url: https://github.com/introlab/rtabmap_ros.git
    version: humble-devel

  external/rtabmap:
    type: git
    url: https://github.com/introlab/rtabmap.git
    version: humble-devel
```
delete old copies
```
rm -rf ~/tmr_ws/src/external/rtabmap_ros
rm -rf ~/tmr_ws/src/external/rtabmap
```
and import
```
vcs import ~/tmr_ws/src < rtabmap_only.repos
```
then fixed `rtabmap_conversions` and `rtabmap_msgs` like before and then i build again
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```
rtabmap_ros is still giving the same error and using ros1 apis instead of ros2

delete rtab folders
```
rm -rf ~/tmr_ws/src/external/rtabmap_ros
rm -rf ~/tmr_ws/src/external/rtabmap
```
clone manually
```
git clone -b humble-devel https://github.com/introlab/rtabmap_ros.git ~/tmr_ws/src/external/rtabmap_ros
git clone -b humble-devel https://github.com/introlab/rtabmap.git ~/tmr_ws/src/external/rtabmap
```
delete all build artifacts to make sure it doenst use the old files
```
rm -rf ~/tmr_ws/build ~/tmr_ws/install ~/tmr_ws/log
```
rebuild 
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```
change code in `rtabmap_ros/rtabmap_conversions/src/MsgConversion.cpp`
replaced
```
pcl::PCLPointCloud2 cloud;
pcl_conversions::toPCL(msg.laser_scan, cloud);
```
with
```
laser_geometry::LaserProjection projector;
sensor_msgs::msg::PointCloud2 cloud_msgs;
projector.projectLaser(msg.laser_scan, cloud_msgs);

pcl::PCLPointCloud2 cloud;
pcl_conversions::toPCL(cloud_msgs, cloud);
```
replaced with the standard, portable ROS 2 approach due to that line not compiling properly. 

Now i rebuild
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --packages-select rtabmap_conversions
```
and it still failed with the same errors

## 2/3/26
forgot to build rtabmap so i will do that and rerun to check if it will work and wont give the pcl errors any moreand i also reverted all the changes i made to  `rtabmap_ros/rtabmap_conversions/src/MsgConversion.cpp`
```
cd ~/tmr_ws/src/external/rtabmap
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
```
cd ~/tmr_ws
rm -rf build/rtabmap_ros build/rtabmap_conversions install/rtabmap_ros install/rtabmap_conversions
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
still failed and i think the code is compiling using the wrong pcl but i need to chekc my files to make sure

turns out my system has ROS1 pcl packages and i need to remove them while keeping the ROS2 modern pcl packages
```
sudo apt remove libpcl-conversions-dev libpcl-ros-dev pcl-ros-tools python3-pcl-ros
```
then i rebuilt again
```
cd ~/tmr_ws
rm -rf build/rtabmap_ros build/rtabmap_conversions install/rtabmap_ros install/rtabmap_conversions
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

removed that whole big error but know i get a different error and it looks like i just need to clone the ROS 2 version of pcl_conversions and ROS2 humble pcl_msgs repo then build
```
cd ~/ros2_humble/src
git clone https://github.com/ros-perception/perception_pcl.git -b humble
git clone https://github.com/ros-perception/pcl_msgs.git -b ros2
```
```
cd ~/ros2_humble
colcon build --symlink-install --packages-select pcl_msgs
colcon build --symlink-install --packages-select pcl_conversions --cmake-args -DBUILD_TESTING=OFF
```
since it all built now i can build the whole thing
```
colcon build --symlink-install
```
The entire ros2_humble built successfuly with no known errors as of now.

clean and rebuild my workspace now
```
cd ~/tmr_ws
rm -rf build install log
colcon build --symlink-install
```

robot_locolization coudn't find geogrpahic_msgs so i have to add it to ros2_humble and rebuild it
```
cd ~/ros2_humble/src
git clone https://github.com/ros-geographic-info/geographic_info.git -b ros2
```
```
cd ~/ros2_humble
colcon build --symlink-install --packages-select geographic_msgs geographic_info geodesy
```

have to add angles to ros2_humble for geographic_msgs to build
```
cd ~/ros2_humble/src
git clone https://github.com/ros/angles.git -b ros2
```
```
cd ~/ros2_humble
colcon build --symlink-install --packages-select angles
colcon build --symlink-install --packages-select geographic_msgs geographic_info geodesy
```

then build the whole thing again
```
colcon build --symlink-install
```

then rebuild workspae for the 100th time
```
cd ~/tmr_ws
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install
```

remove 
```
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
  find_library(
    crypto_LIB NAMES crypto
    PATHS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()
```
from `rtabmap_ros/rtabmap_sync/CMakeLists.txt` because it doenst need to find it and ROS2 will find the correct packages.

remove 
```
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
  find_library(
    rcutils_LIB NAMES rcutils
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
  find_library(
    crypto_LIB NAMES crypto
    PATHS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()
```
from `rtabmap_ros/rtabmap_viz/CMakeLists.txt` because it doenst need to find it and ROS2 will find the correct packages.


remove 
```
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()
```
from `rtabmap_ros/rtabmap_slam/CMakeLists.txt` because it doenst need to find it and ROS2 will find the correct packages.

remove 
```
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    message_filters_LIB NAMES message_filters
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()
```
from `rtabmap_ros/rtabmap_rviz_plugins/CMakeLists.txt` because it doenst need to find it and ROS2 will find the correct packages.

remove 
```
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()
```
from `rtabmap_ros/rtabmap_odom/CMakeLists.txt` because it doenst need to find it and ROS2 will find the correct packages.

then rerun
```
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install
```

forgot to specify to only build external packages and skip rest along with skipping nav2_system_tests sincei dont have gazebo and im not using it
```
source ~/ros2_humble/install/setup.bash
colcon build --packages-skip tmr_base tmr_interfaces tmr_mission tmr_nav tmr_perception tmr_sensors tmr_v5_firmware nav2_system_tests --symlink-install
```

FINALLY EVERYTHING BUILT WITH NO ERRORS there were a couple of warnings but that is to be expected and shouldn't affect my project at all.


