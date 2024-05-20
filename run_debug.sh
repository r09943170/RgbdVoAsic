#!/bin/bash
cd build
make
cd ..
# ./build/liyang-odometry ../../dataset/fr1_360/rgbd_test2frames.txt ./result_test2frames.txt RgbdICP >> debug_Mat.txt
./build/liyang-odometry ../../dataset/fr1_360/rgbd_test2frames.txt ./result_test2frames.txt RgbdICP