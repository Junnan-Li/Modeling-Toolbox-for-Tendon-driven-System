% Ultilities test




clear all
close all
clc
%% rotation transformation


disp('Function test start: euler2R_XYZ.m')

for i = 1:20
    euler = 5*rand(1,3);
    

    R_ult = euler2R_XYZ(euler);

    R_std = eul2rotm(euler,'XYZ');

    error_euler = abs(R_ult-R_std);
    assert(max(error_euler(:)) < 1e-12, 'Class Error: euler Transformation matrix do not match!')
end
disp('Test pass!')


%%

euler1 = [pi,pi/2-0.0001,pi];
euler2 = [0,pi,0];

R11 = euler2R_XYZ(euler1);
R21 = euler2R_XYZ(euler2);
R12 = eul2rotm(euler1,'XYZ');
R22 = eul2rotm(euler2,'XYZ');
euler1 = R2euler_XYZ(R11);
euler2 = rotm2eul(R12,'XYZ');







