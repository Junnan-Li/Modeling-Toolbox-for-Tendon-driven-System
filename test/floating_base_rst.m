

clear all
close all
clc

robot = rigidBodyTree;
robot.DataFormat = 'column';
% robot.Gravity = g;
last_link_name = 'base';

% floating base alternative(6 dof)
transh_1 = [0,0,1,0;1,0,0,0;0,1,0,0;0,0,0,1];
transh_2 = [0,0,1,0;0,1,0,0;-1,0,0,0;0,0,0,1];
transh_3 = [1,0,0,0;0,0,1,0;0,-1,0,0;0,0,0,1];
transh_all = {transh_1,transh_2,transh_3};

roth_1 = [0,0,-1,0;0,1,0,0;1,0,0,0;0,0,0,1];
roth_2 = [1,0,0,0;0,0,-1,0;0,1,0,0;0,0,0,1];
roth_3 = [0,0,1,0;1,0,0,0;0,1,0,0;0,0,0,1];
roth_all = {roth_1,roth_2,roth_3};
%%
for i = 1:3
    link_i_name = char("base_trans"+num2str(i));
    bodyi = rigidBody(link_i_name);
    joint_i_name = char("base_trans_jnt"+num2str(i));
    jnt_i = rigidBodyJoint(joint_i_name,'prismatic');
    setFixedTransform(jnt_i,transh_all{i});
    bodyi.Joint = jnt_i;
    bodyi.Mass = 0;
    bodyi.Inertia = [0,0,0,0,0,0];
    bodyi.CenterOfMass = [0,0,0];
    addBody(robot,bodyi,last_link_name)
    last_link_name = link_i_name;
end

for i = 1:3
    link_i_name = char("base_rot"+num2str(i));
    bodyi = rigidBody(link_i_name);
    joint_i_name = char("base_rot_jnt"+num2str(i));
    jnt_i = rigidBodyJoint(joint_i_name,'revolute');
    setFixedTransform(jnt_i,roth_all{i});
    bodyi.Joint = jnt_i;
    bodyi.Mass = 0;
    bodyi.Inertia = [0,0,0,0,0,0];
    bodyi.CenterOfMass = [0,0,0];
    addBody(robot,bodyi,last_link_name)
    last_link_name = link_i_name;
end

tran_end = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
body_end = rigidBody('endeff');
jnt_end = rigidBodyJoint('jnt_end','fixed');
setFixedTransform(jnt_end,tran_end);
body_end.Joint = jnt_end;
body_end.Mass = 0;
body_end.Inertia = [0,0,0,0,0,0];
body_end.CenterOfMass = [0,0,0];
addBody(robot,body_end,last_link_name)

robot.show([1,.2,.3,1,2,1]')


ik = inverseKinematics('RigidBodyTree',robot);
X_base = rand(6,1);
eePose = [euler2R_XYZ(X_base(4:6)),X_base(1:3); 0 0 0 1];
[configSoln,solnInfo] = ik('endeff',eePose,[1,1,1,1,1,1],robot.homeConfiguration);

J_geom = geometricJacobian(robot,configSoln,'endeff')

J_geom_1 = [J_geom(4:6,:);J_geom(1:3,:)]








