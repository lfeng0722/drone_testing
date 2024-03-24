# Autoland Documentation

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

## Install Autoland

1. Download the attached file and unzip it to get Autoland package.
    
    [skyy_autoland_ws.zip](Autoland%20Documentation%20d09d6717e6b1461fa465d472eade4856/skyy_autoland_ws.zip)
    

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

## Run Autoland with AirSim standalone environment

### Configuration

1. Set up ardupilot: in `marker_landing_system/src/skyy_autoland/launch/autoland_ardupilot.launch`, modify `ardupilot_directory` to your ardupilot path and `sim_address` to 127.0.0.1 if you run Autoland in Ubuntu. 
2. Set up AirSim environment: in `landing_simulation_system/src/skyy_simulation/launch/airsim_sim.launch`, modify `airsim_environment_directory` to your AirSim binary path and `airsim_environment` to AirSim binary name (e.g., `CityParkEnvironmentCollec`, depending on bash file name in AirSim binary folder). set `launch_airsim` as `true`
3. Set all script files in all modules as executable (todo: write a bash script for this step). For example, set `ga_gen.py` in `landing_simulation_system/src/test_generation/scripts` using the following commands:
    
    ```bash
    cd landing_simulation_system/src/test_generation/scripts
    chmod +x ga_gen.py
    ```
    

### Run simulation

1. Open a terminal and enter `marker_landing_system`, run following commands:
    
    ```bash
    source devel/setup.bash
    roslaunch skyy_autoland autoland_ardupilot.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
    ```
    
2. Open a terminal and enter `landing_simulation_system`, run following commands:
    
    ```bash
    source devel/setup.bash
    roslaunch skyy_simulation airsim_sim.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
    ```
    
    It may take longer time to launch AirSim when running the launch file first time. The AirSim window would be stuck at black screen and the AirSim ROS connection would be also stuck in terminal output. Wait for AirSim window  loading completely and then kill the roslaunch process in terminal. Run airsim_sim.launch again to normally launch AirSim simulation.
    
3. Open a terminal and enter `landing_simulation_system`, run following commands:

```bash
source devel/setup.bash
rosrun test_generation ga_gen.py
```

# Customized AirSim Environment in Unreal

### Build Unreal 4.27

Follow the [instruction](https://microsoft.github.io/AirSim/build_linux/#build-unreal-engine) to build Unreal Engine

## Build AirSim Environment

1. Download CityPark Unreal project from the link and unzip: https://drive.google.com/file/d/1hpbVL82nywqJTU7tLjtDZgGT2HXWp08v/view?usp=sharing
2. Open a terminal in Unreal Engine folder and run the following command:
    
    ```bash
    ./GenerateProjectFiles.sh /path/to/CityPark/CityParkEnvironmentCollec.uproject -game -engine
    ```
    
3. Open the CityPark folder in VSCODE
4. Run the project in VSCODE in DebugGame mode

## Run Autoland with AirSim in Unreal

### Configuration

1. set `launch_airsim` as `false` in `landing_simulation_system/src/skyy_simulation/launch/airsim_sim.launch`
2. Copy the file `settings_ardupilot.json` in `marker_landing_system/src/skyy_autoland/config` to `~/Documents/AirSim` and rename it to `settings.json`
3. Set up ardupilot: in `marker_landing_system/src/skyy_autoland/launch/autoland_ardupilot.launch`, modify `ardupilot_directory` to your ardupilot path and `sim_address` to 127.0.0.1 if you run Autoland in Ubuntu. 

### Run Simulation

1. Open a terminal and enter `marker_landing_system`, run the following commands:
    
    ```bash
    source devel/setup.bash
    roslaunch skyy_autoland autoland_ardupilot.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
    ```
    
2. Run AirSim in Unreal Engine following steps 3 and 4 in Section *Build AirSim Environment.* 
3. Open a terminal and enter `landing_simulation_system`, run the following commands:
    
    ```bash
    source devel/setup.bash
    roslaunch skyy_simulation airsim_sim.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
    ```
    
4. Open a terminal and enter `landing_simulation_system`, run following commands:
    
    ```bash
    source devel/setup.bash
    rosrun test_generation ga_gen.py
    ```
    

ImportError: /lib/libgdal.so.26: undefined symbol: TIFFReadRGBATileExt, version LIBTIFF_4.0

## Working notes for clean custom build

Prepare Unreal Engine:

```bash
# Complete account sign-up and GitHub/Epic account link
cd ~
git clone -b 4.27 [git@github.com](mailto:git@github.com):EpicGames/UnrealEngine.git --depth=1
cd UnrealEngine/

# Build
./Setup.sh
./GenerateProjectFiles.sh
make

# Had to do this to get the editor to launch
./Engine/Build/BatchFiles/Linux/Build.sh ShaderCompileWorker Linux Development

# Launch editor (first time launch may take some time
./Engine/Binaries/Linux/UE4Editor
```

Prepare AirSim:

```bash
cd ~
git clone https://github.com/Microsoft/AirSim.git --depth=1
cd AirSim

# Build
./setup.sh
./build.sh

# Prepare environment
# For some reason the environment must be built first before openning in editor
# See:
../UnrealEngine/GenerateProjectFiles.sh $(realpath ./Unreal/Environments/Blocks/Blocks.uproject) -game -engine
cd ./Unreal/Environments/Blocks
make BlocksEditor
```

Basic `~/Document/AirSim/settings.json`:

```bash
{
	"Vehicles": {
	    "SimpleFlight": {
	      "VehicleType": "SimpleFlight"
	    }
	}
}
```

Complete ArduCopter `~/Document/AirSim/settings.json`:

```bash
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "OriginGeopoint": {
      "Latitude": -35.363261,
      "Longitude": 149.165230,
      "Altitude": 583
    },
    "Vehicles": {
      "Copter": {
        "VehicleType": "ArduCopter",
        "UseSerial": false,
        "LocalHostIp": "127.0.0.1",
        "UdpIp": "127.0.0.1",
        "UdpPort": 9003,
        "ControlPort": 9002,
        "AutoCreate": true,
        "Sensors": {
          "Imu": {
            "SensorType": 2,
            "Enabled": true
          },
          "Gps": {
            "SensorType": 3,
            "Enabled": true
          },
          "Lidar1": {
            "SensorType": 6,
            "Enabled": true,
            "NumberOfChannels": 1,
            "PointsPerSecond": 5000,
            "DrawDebugPoints": false,
            "RotationsPerSecond": 10,
            "VerticalFOVUpper": 0,
            "VerticalFOVLower": 0,
            "HorizontalFOVStart": 0,
            "HorizontalFOVEnd": 359,
            "DataFrame": "SensorLocalFrame",
            "ExternalController": true
          }
        },
        "Cameras": {
          "downward_custom": {
            "CaptureSettings": [
              {
                "PublishToRos": 1,
                "ImageType": 0,
                "Width": 640,
                "Height": 480,
                "FOV_Degrees": 27,
                "DepthOfFieldFstop": 2.8,
                "DepthOfFieldFocalDistance": 200.0, 
                "DepthOfFieldFocalRegion": 200.0,
                "TargetGamma": 1.5
              }
            ],
            "X": 0, "Y": 0, "Z": 0,
            "Pitch": -90, "Roll": 0, "Yaw": 0
          }
        }
      }
    },
    "SubWindows": [
      {"WindowID": 0, "ImageType": 0, "CameraName": "downward_custom", "Visible": true}
    ]
}
```



Option 1: Run project editor:

```bash
cd ~
./UnrealEngine/Engine/Binaries/Linux/UE4Editor
# Select ./Airsim/Unreal/Environments/Blocks/Blocks.uproject
```

Option 2: Compile directly from the CLI:

```bash
cd ~

# Patch
nano ./UnrealEngine/Engine/Source/Runtime/Core/Public/Templates/TypeHash.h
# Add: #include <stdint.h>

# Compile
# Note: first build will take a long time
# Note: set clientconfig to "Shipping" if you do not want to include debug files
./UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -project="$(realpath ./AirSim/Unreal/Environments/Blocks/Blocks.uproject)" -noP4 -platform=Linux -clientconfig=Development -cook -allmaps -build -stage -pak -archive -archivedirectory=./

# Run
cd ~/AirSim/Unreal/Environments/Blocks/Saved/StagedBuilds/LinuxNoEditor
./Blocks.sh
```

Build the AirSim ROS packages:

```bash
cd ~/AirSim/ros
catkin build
source devel/setup.bash
# Automaitcally source:
# echo 'source ~/AirSim/ros/devel/setup.bash' >> ~/.bashrc
```

ROS Packages

Dependencies:

```bash
# Required ROS packages
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras

# ROS development packages
sudo apt install \
	\ # Repos
	git python3-vcstool \
	\ # FUEL planner requirements
	ros-noetic-nlopt libdw-dev
```

If you are getting errors like:

```bash
** WARNING ** io features related to ...
```

Then you can comment out the following line (ln. ~364) to mute it in `/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake`:

```bash
pcl_message("** WARNING ** ${_component} features related to ${_lib} will be disabled")
```

## PX4 SITL

Toolchain setup (adapted from [official source](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)):

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git px4 --recursive --branch v1.11.3 --depth 1

cd ~/px4
bash ./Tools/setup/ubuntu.sh

# The following is done automatically on the clone command above
# git checkout v1.11.3 # or latest stable release 
# git submodule update --init --recursive

make px4_sitl_default
```

## Mapping

```jsx
sudo apt install ros-noetic-octomap-mapping ros-noetic-octomap-rviz-plugins
```

Planning

## Run the Simulation

### Startup

The `px4` and `airsim`  environments should be started and left running at all times: 

```bash
cd ~/px4
make px4_sitl_default none_iris
```

```bash
cd ~/AirSim/Unreal/Environments/Blocks/Saved/StagedBuilds/LinuxNoEditor
./Blocks.sh
```

ROS nodes can be run and restarted in any order (once the `roscore`  is up)

```bash
roscore
# In another window
# Prep the rest of ROS to rely on simtime
# Typically this would be done at the top of a master launch file
# But it's easier to do with a specifc roscore running if manually launching everything
rosparam set /use_sim_time true
```

```bash
# source ~/AirSim/ros/devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch publish_clock:=true
```

```bash
# Start mavros on a secondary UDP port
# This allows automatic connection directly to PX4 with QGroundControl like a real system would
roslaunch mavros px4.launch fcu_url:=udp://:14540@:14580
# TODO: This needs to use a custom configuration to enable local pose output to TF2
```

```bash
# Fix the weird rotation from AirSim
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 3.141592653 base_link world_ned
```

```bash
# TODO: Custom launch file with remappings and octomap settings 
roslaunch octomap_mapping.launch
```

```bash
# Use plugins:
# - TF
# - Point cloud (for laser scan)
# - Octomap visualisation (for occupied grids
rviz
```

### Resetting

```bash
# Reset AirSim with API/[Backspace]
# At the same time, trigger a force disarm: https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
rosservice call /airsim_node/reset "waitOnLastTask: false" && \
	rosservice call /mavros/cmd/command "{broadcast: false, command: 400, confirmation: 1, param1: 0, param2: 21196, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}" && \
	rosservice call /octomap/octomap_server/reset "{}"
```

After a few seconds, the drone should re-calibrate it’s sensors/state and be ready for take-off again

### Commanding

```bash
rosrun mavros mavsafety arm
rosrun mavros mavcmd takeoffcur 0 0 4

```