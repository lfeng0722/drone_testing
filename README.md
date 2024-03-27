# GARL Documentation

# Installation

## Prerequisite

### Ardupilot

Follow the [instruction](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux) to install Ardupilot SITL

### AirSim Source Code

Follow the [instruction](https://microsoft.github.io/AirSim/build_linux/) to build AirSim. This step is for setting AirSim ROS package in Autoland later.

### ROS packages

- ROS-noetic
- MAVROS
- RVIZ
- Other missing ROS packages when compiling Autoland (to update)

### AirSim environment binaries

Download AirSim binaries from the [link](https://drive.google.com/drive/folders/1BGHT6QWqC53SEXPojBrTYvwlCBGn3Hv9) and unzip it.

### AirSim Python SDK

[airsim_python_sdk.zip](Autoland%20Documentation%20d09d6717e6b1461fa465d472eade4856/airsim_python_sdk.zip)

1. Download the file and unzip it.
2. Open a terminal in the unzipped folder
3. run the command ‘pip install -e .’ to install AirSim library in Python


## Simulation map
1. [Download our packaged simulation map 
](https://drive.google.com/file/d/1Z8ika2kn8KrnMIwiAt2WGTQ13_62vpeZ/view?usp=sharing)
2. Unzip it


## Install Landing System and Testing Method

1. Clone the repo.


Before compilation,  you must remove a conflicting ROS package:

```bash
sudo apt-get remove ros-noetic-multi-map-server
```

1. Open a terminal and enter the folder skyy_autoland_ws/marker_landing_system, run following commands

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

1. Open the file skyy_autoland_ws/landing_simulation_system/landing_simulation_system/src/airsim_ros/src/airsim_ros_pkgs/CMakeLists.txt in a text editor, modify line 5 to set AIRSIM_ROOT as your AriSim path  
2. Open another terminal and enter the folder skyy_autoland_ws/landing_simulation_system, run following commands

```bash
source ../marker_landing_system/devel/setup.bash
catkin_make
```

## Run the auto-landing system and our test method

### Configuration

1. Set up ardupilot: in `src/skyy_autoland/launch/autoland_ardupilot.launch`, modify `ardupilot_directory` to your ardupilot path, modify `airsim_environment_directory` to the simulation map's address.  
2. Set up the map in `src/simulation/scripts/airsim_sitl`, modify `~PlayerStartTag`, and enter the name of map `court` or `lawn`.
3. Set all script files in all modules as executable. For example, set `run.py` in `src/test_generation/scripts` using the following commands:
    
    ```bash
    cd src/test_generation/scripts
    chmod +x run.py
    ```
    

### Run simulation

1. Open a terminal, run following commands:
    
    ```bash
    source devel/setup.bash
    roslaunch skyy_autoland autoland_ardupilot.launch
    ```
2. After 3 `connected` pop up, start test case generation

###  Test case generation
1. Set up the folder to save the record in the `run.py`.
2. Set up the map
3. Open a terminal, run following commands:

```bash
source devel/setup.bash
rosrun test_generation rlaga_gen.py
```

### Evaluation
1. Modify the result folder address `exp_folder` in `Evaluation.py`
2. Run the `Evaluation.py`

