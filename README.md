# EEE4610_visual_navigation

## Overview

This repository is for EEE4610 Electrical and Electronic Engineering Capstone Design @ dept. of Electrical and Electronic Engineering, Yonsei University

* Author: **Team Overcoming**

## Installation

### 0. git SSH

Make sure you have a git SSH key. [How to generate SSH key]

### 1. Clone repository

```bash
mkdir ~/EEE4610_visual_navigation
cd ~/EEE4610_visual_navigation
git clone --recurse-submodules -b sensor_integration git@github.com:poodlee/EEE4610_visual_navigation.git .
```

(The dot at the end (.) is important)

### 2. Install dependencies

```bash
# catkin dep.
sudo apt-get install python3-catkin-tools python3-vcstool python3-osrf-pycommon
# system dep.
sudo apt-get install libglew-dev libopencv-dev libyaml-cpp-dev 
# Ceres dep.
sudo apt-get install libblas-dev liblapack-dev libsuitesparse-dev
```

You perhaps have to install [Ceres Solver] maunally.

### 3. SVO_Pro setup

```bash
cd ~/EEE4610_visual_navigation
catkin config --init --mkdirs --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
cd src/rc_car/svo_pro
vcs-import < ./rpg_svo_pro_open/dependencies.yaml
cd rpg_svo_pro_open/svo_online_loopclosing/vocabularies && ./download_voc.sh
```

### 4. GTSAM setup

```bash
cd ~/EEE4610_visual_navigation/src/rc_car/gtsam
mkdir build && cd build
cmake ..
sudo make install
```

You might have to wait for quite considerable time to get it done(~30min). After it finishes, head to `/usr/local/lib` and check if `libgtsam.so` file exists.

### 5. Build entire workspace
```bash
cd ~/EEE4610_visual_navigation
catkin build
```

### 6. Source ROS and workspace setup files

```bash
source /opt/ros/noetic/setup.bash
cd ~/EEE4610_visual_navigation
source devel/setup.bash
```

Every time you open a new terminal, you have to run the commands above.

Otherwise, you can modify `.bashrc` file.

```bash
gedit ~/.bashrc
```

Append the lines below at the end of the file.
```bash
source /opt/ros/noetic/setup.bash
source ~/EEE4610_visual_navigation/devel/setup.bash
```

`.bashrc` file will automatically execute the commands in it every time you open a new terminal.

## Usage

On RC car: 
```bash
roslaunch rc_main rc_main.launch
```

On laptop:
```bash
roslaunch laptop_main laptop_main.launch
```

## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the crt dec -->
ontent of this config file

## Launch files

* **rc_main.launch:** Launch sensor-to-msg nodes and svo_pro node.

  <!-- * **`argument_1`** Short dec -->

* **sensor.launch:** Launch only sensor-to-msg nodes.

  <!-- * **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`. -->

* **slam.launch:** Launch only svo_pro node; for rosbag testing.

  <!-- * **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`. -->

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.

#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

    The temperature measurements from which the average is computed.

#### Published Topics

...

#### Services

* **`get_average`** ([std_srvs/Trigger])

    Returns information about the current average.

#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

    The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

    The size of the cache.

### NODE_B_NAME

...

[How to generate SSH key]: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
[Ceres Solver]: http://ceres-solver.org/installation.html
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html