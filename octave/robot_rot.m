function R = robot_rot(A, B, C)
  R = rotz(C) * roty(B) * rotx(A);
end
