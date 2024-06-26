#!/bin/bash
cd build
make
cd ..
./build/liyang-odometry ../../dataset/fr1_360/rgbd_assoc.txt ./result/pose_fr1_360.txt RgbdICP
./build/liyang-odometry ../../dataset/fr1_desk/rgbd_assoc.txt ./result/pose_fr1_desk.txt RgbdICP
./build/liyang-odometry ../../dataset/fr1_desk2/rgbd_assoc.txt ./result/pose_fr1_desk2.txt RgbdICP
./build/liyang-odometry ../../dataset/fr1_floor/rgbd_assoc.txt ./result/pose_fr1_floor.txt RgbdICP
./build/liyang-odometry ../../dataset/fr1_room/rgbd_assoc.txt ./result/pose_fr1_room.txt RgbdICP
./build/liyang-odometry ../../dataset/fr1_rpy/rgbd_assoc.txt ./result/pose_fr1_rpy.txt RgbdICP
./build/liyang-odometry ../../dataset/fr1_xyz/rgbd_assoc.txt ./result/pose_fr1_xyz.txt RgbdICP
#./rgbd-odometry-liyang ../../dataset/fr2_360_hemisphere/rgbd_assoc.txt ./result/pose_fr2_360_hemisphere.txt RgbdICP
#./rgbd-odometry-liyang ../../dataset/fr2_360_kidnap/rgbd_assoc.txt ./result/pose_fr2_360_kidnap.txt RgbdICP
#./rgbd-odometry-liyang ../../dataset/fr2_desk/rgbd_assoc.txt ./result/pose_fr2_desk.txt RgbdICP
#./rgbd-odometry-liyang ../../dataset/fr2_large_no_loop/rgbd_assoc.txt ./result/pose_fr2_large_no_loop.txt RgbdICP
#./rgbd-odometry-liyang ../../dataset/fr2_large_with_loop/rgbd_assoc.txt ./result/pose_fr2_large_with_loop.txt RgbdICP
#./rgbd-odometry-liyang ../../dataset/fr2_rpy/rgbd_assoc.txt ./result/pose_fr2_rpy.txt RgbdICP
#./rgbd-odometry-liyang ../../dataset/fr2_xyz/rgbd_assoc.txt ./result/pose_fr2_xyz.txt RgbdICP
#python3 vo.py ../dataset/fr3_long_office_household/rgbd_assoc.txt ./result/pose_fr3_long_office_household.txt RgbdICP
#python3 vo.py ../dataset/fr3_nostructure_notexture_far/rgbd_assoc.txt ./result/pose_fr3_nostructure_notexture_far.txt RgbdICP
#python3 vo.py ../dataset/fr3_nostructure_texture_far/rgbd_assoc.txt ./result/pose_fr3_nostructure_texture_far.txt RgbdICP
#python3 vo.py ../dataset/fr3_nostructure_texture_near_withloop/rgbd_assoc.txt ./result/pose_fr3_nostructure_texture_near_withloop.txt RgbdICP
#python3 vo.py ../dataset/fr3_structure_notexture_far/rgbd_assoc.txt ./result/pose_fr3_structure_notexture_far.txt RgbdICP
#python3 vo.py ../dataset/fr3_structure_notexture_near/rgbd_assoc.txt ./result/pose_fr3_structure_notexture_near.txt RgbdICP
#python3 vo.py ../dataset/fr3_structure_texture_far/rgbd_assoc.txt ./result/pose_fr3_structure_texture_far.txt RgbdICP
#python3 vo.py ../dataset/fr3_structure_texture_near/rgbd_assoc.txt ./result/pose_fr3_structure_texture_near.txt RgbdICP
