How to calibrate the transformation between hand and eye:

Preparations:
* place the camera on the gripper
* place a checkerboard somewhere visible to the camera and fix it
* move the robot arm around the checkerboard and write down some joint poses 
  as calibration path in data/calibration_path.yaml (minimum 4),
  be sure that the checkerboard is fully visible inside the camera image in each
  calibration pose


Run software:
* roscore
* rosrun arm_control arm_control_node
* roslaunch fugu_configurations camera_flex_narrow_image_proc.launch
* rosrun pattern_pose_estimation checkerboard_detector_node _rows:=4 _cols:=6 _size:=0.04 _calibrated:=true image:=/forward_stereo_camera/left/image_rect
* rosrun rviz rviz
You now should see poses of the checkerboard and the gripper in rviz when you
display the transform hierarchy.

!!! PLACE YOUR HAND ON THE EMERGENCY STOP OF THE ROBOT BEFORE THE NEXT STEP !!!

* rosrun arm_camera_calibration arm_camera_calibration data/calibration_path.yaml
If everything works fine, the calibration procedure should steer the robot into
the poses that are given in calibration_path.yaml and write pattern and arm poses
to two files in arm_camera_calibration/: pattern_poses.txt and arm_poses.txt.
When all poses are written you can shut down the arm_camera_calibration node.

* octave octave/calibration.m
This runs the transformation computation using the written arm_poses.txt and
pattern_poses.txt
The transformation is printed on the screen. Use the last column of the matrix
as translation and the RPY values for the rotation to configure the 
hand eye tranformation as parameters in the arm_control package.

