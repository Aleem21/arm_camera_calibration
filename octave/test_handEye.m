world_to_base = randtrans();
gripper_to_camera = randtrans();

M = 15;

bHg = zeros(4,4,M);
wHc = zeros(4,4,M);
cHw = zeros(4,4,M);
for i = 1:M
    base_to_gripper = randtrans();
    world_to_camera = world_to_base * base_to_gripper * gripper_to_camera;
    noise_m = zeros(4,4);
    noise_v = 0.0001 * ones(4,4);
    noise = normal_rnd(noise_m, noise_v);
    bHg(:,:,i) = base_to_gripper; % + normal_rnd(noise_m, noise_v);
    wHc(:,:,i) = world_to_camera;
    cHw(:,:,i) = inv(world_to_camera);
end

% delta gripper
d_g = zeros(4,4,M-1);
% delta camera
d_c = zeros(4,4,M-1);

for i = 2:M
  d_g(:,:,i) = inv(bHg(:,:,i-1)) * bHg(:,:,i);
  d_c(:,:,i) = inv(wHc(:,:,i-1)) * wHc(:,:,i);
end;

gHc = handEye(bHg, wHc);
gHc2 = handEye2(bHg, cHw);
% cal = extrinsic_calibration(d_g, d_c);

gripper_to_camera
gHc
gHc2
% cal

