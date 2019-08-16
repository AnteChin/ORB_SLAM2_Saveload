# Introduce
This is SLAM project based on ORB_SLAM2 and running for drone system. Before testing this version of ORB_SLAM2, please make sure that you have installed dependencies that required by ORB_SLAM2. To install this project, run `build.sh`.

This project includes following directories:

* `src` includes all source files running for slam.
* `include` includes header files
* `ROS` package includes the ros program. ZED camera need to read data from a ros node, so please make sure you ros program in running before you run this program.
* `Documentation` includes an introduction to this project. I am really recommend you to read it if you want to edit codes inside it.

This is project is finished in UC Davis, VR lab.

## Dependencies

### Sophus

Besides the libaries required by ORB_SLAM2, you need to install `Sophus` because it's what we used to reprensent Lie group.

```bash
# bash command for installing Sophus
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
make
sudo make install
```

## Features
1. All basic ORB_SLAM2 features.
2. Save and load binary map file.

## Example

To run the Stereo example, you need to use ROS.
1. Open Terminal and run
```bash
roscore
```
2. Open a new Terminal and run
```
source catkin_ws/devel_isolated/setup.bash
roslaunch zed_wrapper zed.launch
```
3. Open a new Terminal and run in the project directory
```bash
rosrun Drone_SLAM Stereo /home/quadcopterar/Research/Save_Load/Vocabulary/ORBvoc.bin /home/quadcopterar/Research/Save_Load/zedSetting.yaml false
```

