% Ultilities test




clear all
close all
clc
%% rotation transformation


disp('Function test start: euler2R_XYZ.m')
euler = rand(1,3);

R_ult = euler2R_XYZ(euler);

R_std = eul2rotm(euler,'XYZ');

error_euler = abs(R_ult-R_std);
assert(max(error_euler(:)) < 1e-12, 'Class Error: euler Transformation matrix do not match!')

disp('Test pass!')





