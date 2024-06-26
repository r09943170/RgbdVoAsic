#python ../evaluate_rpe.py ../../dataset/fr1_360/groundtruth.txt ./result/pose_fr1_360.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr1_360.txt
#python ../evaluate_rpe.py ../../dataset/fr1_desk/groundtruth.txt ./result/pose_fr1_desk.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr1_desk.txt
#python ../evaluate_rpe.py ../../dataset/fr1_desk2/groundtruth.txt ./result/pose_fr1_desk2.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr1_desk2.txt
#python ../evaluate_rpe.py ../../dataset/fr1_floor/groundtruth.txt ./result/pose_fr1_floor.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr1_floor.txt
#python ../evaluate_rpe.py ../../dataset/fr1_room/groundtruth.txt ./result/pose_fr1_room.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr1_room.txt
#python ../evaluate_rpe.py ../../dataset/fr1_rpy/groundtruth.txt ./result/pose_fr1_rpy.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr1_rpy.txt
#python ../evaluate_rpe.py ../../dataset/fr1_xyz/groundtruth.txt ./result/pose_fr1_xyz.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr1_xyz.txt

python ./evaluate_rpe.py ../../dataset/fr1_360/groundtruth.txt ./result/pose_fr1_360.txt --fixed_delta --scale 0.0002 --delta_unit f --verbose > ./result/rpe_fr1_360.txt
python ./evaluate_rpe.py ../../dataset/fr1_desk/groundtruth.txt ./result/pose_fr1_desk.txt --fixed_delta --scale 0.0002 --delta_unit f --verbose > ./result/rpe_fr1_desk.txt
python ./evaluate_rpe.py ../../dataset/fr1_desk2/groundtruth.txt ./result/pose_fr1_desk2.txt --fixed_delta --scale 0.0002 --delta_unit f --verbose > ./result/rpe_fr1_desk2.txt
python ./evaluate_rpe.py ../../dataset/fr1_floor/groundtruth.txt ./result/pose_fr1_floor.txt --fixed_delta --scale 0.0002 --delta_unit f --verbose > ./result/rpe_fr1_floor.txt
python ./evaluate_rpe.py ../../dataset/fr1_room/groundtruth.txt ./result/pose_fr1_room.txt --fixed_delta --scale 0.0002 --delta_unit f --verbose > ./result/rpe_fr1_room.txt
python ./evaluate_rpe.py ../../dataset/fr1_rpy/groundtruth.txt ./result/pose_fr1_rpy.txt --fixed_delta --scale 0.0002 --delta_unit f --verbose > ./result/rpe_fr1_rpy.txt
python ./evaluate_rpe.py ../../dataset/fr1_xyz/groundtruth.txt ./result/pose_fr1_xyz.txt --fixed_delta --scale 0.0002 --delta_unit f --verbose > ./result/rpe_fr1_xyz.txt

#python ../evaluate_rpe.py ../../dataset/fr2_360_hemisphere/groundtruth.txt ./result/pose_fr2_360_hemisphere.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr2_360_hemisphere.txt
#python ../evaluate_rpe.py ../../dataset/fr2_360_kidnap/groundtruth.txt ./result/pose_fr2_360_kidnap.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr2_360_kidnap.txt
#python ../evaluate_rpe.py ../../dataset/fr2_desk/groundtruth.txt ./result/pose_fr2_desk.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr2_desk.txt
#python ../evaluate_rpe.py ../../dataset/fr2_large_no_loop/groundtruth.txt ./result/pose_fr2_large_no_loop.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr2_large_no_loop.txt
#python ../evaluate_rpe.py ../../dataset/fr2_large_with_loop/groundtruth.txt ./result/pose_fr2_large_with_loop.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr2_large_with_loop.txt
#python ../evaluate_rpe.py ../../dataset/fr2_rpy/groundtruth.txt ./result/pose_fr2_rpy.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr2_rpy.txt
#python ../evaluate_rpe.py ../../dataset/fr2_xyz/groundtruth.txt ./result/pose_fr2_xyz.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr2_xyz.txt
#python evaluate_rpe.py ./dataset/fr3_long_office_household/groundtruth.txt ./result/pose_fr3_long_office_household.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr3_long_office_household.txt
#python evaluate_rpe.py ./dataset/fr3_nostructure_notexture_far/groundtruth.txt ./result/pose_fr3_nostructure_notexture_far.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr3_nostructure_notexture_far.txt
#python evaluate_rpe.py ./dataset/fr3_nostructure_texture_far/groundtruth.txt ./result/pose_fr3_nostructure_texture_far.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr3_nostructure_texture_far.txt
#python evaluate_rpe.py ./dataset/fr3_nostructure_texture_near_withloop/groundtruth.txt ./result/pose_fr3_nostructure_texture_near_withloop.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr3_nostructure_texture_near_withloop.txt
#python evaluate_rpe.py ./dataset/fr3_structure_notexture_far/groundtruth.txt ./result/pose_fr3_structure_notexture_far.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr3_structure_notexture_far.txt
#python evaluate_rpe.py ./dataset/fr3_structure_notexture_near/groundtruth.txt ./result/pose_fr3_structure_notexture_near.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr3_structure_notexture_near.txt
#python evaluate_rpe.py ./dataset/fr3_structure_texture_far/groundtruth.txt ./result/pose_fr3_structure_texture_far.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr3_structure_texture_far.txt
#python evaluate_rpe.py ./dataset/fr3_structure_texture_near/groundtruth.txt ./result/pose_fr3_structure_texture_near.txt --fixed_delta --delta_unit f --verbose > ./result/rpe_fr3_structure_texture_near.txt
