function [tm, td, rm, rd] = check_calibration(gripper_to_camera, arm_poses, pattern_poses)
  num_poses = size(arm_poses, 3);
  all_translations = zeros(num_poses, 3);
  all_angles = zeros(num_poses, 3);
  for i = 1:num_poses
    gripper_to_pattern = arm_poses(:,:,i) * gripper_to_camera * pattern_poses(:,:,i);
    YPR = tr2rpy(gripper_to_pattern) / pi * 180;
    all_angles(i,:) = YPR;
    t = gripper_to_pattern(1:3, 4);
    all_translations(i,:) = t;
  end
  rm = mean(all_angles);
  tm = mean(all_translations);

  rd = sqrt(var(all_angles));
  td = sqrt(var(all_translations));
end
