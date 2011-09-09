if (nargin < 2)
    printf("Usage: %s <poses file> <poses file> <poses file> ... ", program_name());
    exit;
end

arg_list = argv ();

tmp_poses = read_transformations(arg_list{1});
num_poses = size(tmp_poses,3);
num_runs = nargin;
poses = zeros(4, 4, num_poses, num_runs);

for i = 1:nargin
    poses(:,:,:,i) = read_transformations(arg_list{i});
end

for i = 1:num_poses
    xvals = poses(1,4,i,:)
    yvals = poses(2,4,i,:)
    zvals = poses(3,4,i,:)
end



#RPYdeg = tr2rpy(gripper_to_camera) / pi * 180.0;

#printf("(X, Y, Z, R, P, Y): (%f, %f, %f, %f, %f, %f)",
#        gripper_to_camera(1,4), gripper_to_camera(2,4), gripper_to_camera(3,4), 
#        RPYdeg(3), RPYdeg(2), RPYdeg(1));
