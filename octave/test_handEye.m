world_to_base = randtrans();
gripper_to_camera = randtrans();

M = 15;

bHg = zeros(4,4,M);
wHg = zeros(4,4,M);
for i = 1:M
    base_to_gripper = randtrans();
    world_to_camera = world_to_base * base_to_gripper * gripper_to_camera;
    noise_m = zeros(4,4);
    noise_v = 0.0001 * ones(4,4);
    noise = normal_rnd(noise_m, noise_v);
    bHg(:,:,i) = base_to_gripper; # + normal_rnd(noise_m, noise_v);
    wHc(:,:,i) = world_to_camera;
end

gHc = handEye(bHg, wHc);

gripper_to_camera
gHc

