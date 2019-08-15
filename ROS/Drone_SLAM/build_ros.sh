echo "Building ROS nodes"

cd ROS/Drone_SLAM
rm -rf build
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
