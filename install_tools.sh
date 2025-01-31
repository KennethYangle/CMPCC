SH_DIR=$(realpath .)

# install dependencies
sudo apt-get install ros-${ROS_DISTRO}-video-stream-opencv
sudo apt-get install libyaml-cpp-dev
sudo apt-get install libarmadillo-dev

# install osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install

# install Eigen
cd ${SH_DIR}/Eigen
mkdir build
cd build
cmake ..
make
sudo make install