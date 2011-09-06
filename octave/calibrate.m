pattern_poses_file = fopen("pattern_poses.txt", "r");
pattern_poses = fscanf(pattern_poses_file, "%f", Inf);
arm_poses_file = fopen("arm_poses.txt", "r");
arm_poses = fscanf(arm_poses_file, "%f", Inf);

assert(rows(pattern_poses), rows(arm_poses));

pattern_poses = reshape(pattern_poses, 4, 4, rows(pattern_poses) / 16);
arm_poses = reshape(arm_poses, 4, 4, rows(arm_poses) / 16);

gHc = handEye2(arm_poses, pattern_poses)

RPY = tr2rpy(gHc)
RPYdeg = RPY / pi * 180.0

