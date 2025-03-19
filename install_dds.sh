#!/bin/bash

## Author: zzm

##install ros
mypassword="123456"


##install dds
echo $mypassword |sudo -S apt-get install -y libasio-dev

cd $HOME
git clone https://gitee.com/greymaner/Fast-CDR.git 
cd Fast-CDR
mkdir build && cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON
echo $mypassword |sudo -S cmake --build . --target install

cd $HOME
git clone https://gitee.com/greymaner/foonathan_memory_vendor.git
cd foonathan_memory_vendor 
mkdir build && cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON
echo $mypassword |sudo -S cmake --build . --target install

cd $HOME
git clone https://gitee.com/greymaner/Fast-DDS.git
cd Fast-DDS
mkdir build && cd build 
cmake ..  -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON
echo $mypassword |sudo -S cmake --build . --target install
