%  generate a dynamic model using the Robotic System Toolbox 


close all
clear all
clc


%% build the tree structure model

rst_tree = rigidBodyTree;
rst_tree.DataFormat = 'column';
rst_tree.Gravity = [0 9.81 0];

%% generate the wrist


body_hand = rigidBody('hand');
jointname = '1';
jnt1 = rigidBodyJoint(jointname,'revolute');
W_T_base = [eye(3),[1;0;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body_hand.Joint = jnt1;
body_hand.Mass = 1; % first body (virtual)
addBody(rst_tree,body_hand,'base');

body_hand = rigidBody('hand2');
jointname = '2';
jnt1 = rigidBodyJoint(jointname,'revolute');
W_T_base = [euler2R_XYZ([pi/2,0,0]),[0;0;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body_hand.Joint = jnt1;
body_hand.Mass = 1; % first body (virtual)
addBody(rst_tree,body_hand,'hand');


%% build fingers

body11 = rigidBody('finger11');
jointname = 'b11';
jnt1 = rigidBodyJoint(jointname,'revolute');
W_T_base = [euler2R_XYZ([-pi/2,0,0]),euler2R_XYZ([-pi/2,0,0])*[1;1;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body11.Joint = jnt1;
body11.Mass = 2; % first body (virtual)
body11.CenterOfMass = [0.5;0;0];
addBody(rst_tree,body11,'hand2');

body12 = rigidBody('finger12');
jointname = '1112';
jnt1 = rigidBodyJoint(jointname,'revolute');
W_T_base = [eye(3),[1;0;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body12.Joint = jnt1;
body12.Mass = 5; % first body (virtual)
% body1.Inertia = inertia_all_rst(:,2);
body12.CenterOfMass = [1;0;0];
addBody(rst_tree,body12,'finger11');

% finger 2
body21 = rigidBody('finger21');
jointname = 'b21';
jnt1 = rigidBodyJoint(jointname,'revolute');
W_T_base = [euler2R_XYZ([-pi/2,0,0]),euler2R_XYZ([-pi/2,0,0])*[1;-1;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body21.Joint = jnt1;
body21.Mass = 1; % first body (virtual)
% body1.Inertia = inertia_all_rst(:,2);
% body1.CenterOfMass = com_all(:,2);
addBody(rst_tree,body21,'hand2');

body22 = rigidBody('finger22');
jointname = '2122';
jnt1 = rigidBodyJoint(jointname,'revolute');
W_T_base = [eye(3),[1;0;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body22.Joint = jnt1;
body22.Mass = 1; % first body (virtual)
% body1.Inertia = inertia_all_rst(:,2);
% body1.CenterOfMass = com_all(:,2);
addBody(rst_tree,body22,'finger21');

%
body31 = rigidBody('finger31');
jointname = 'b31';
jnt1 = rigidBodyJoint(jointname,'revolute');
W_T_base = [euler2R_XYZ([-pi/2,0,0]),euler2R_XYZ([-pi/2,0,0])*[2;0;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body31.Joint = jnt1;
body31.Mass = 0; % first body (virtual)
body31.CenterOfMass = [1;0;0];
addBody(rst_tree,body31,'hand2');

body32 = rigidBody('finger32');
jointname = '3132';
jnt1 = rigidBodyJoint(jointname,'revolute');
W_T_base = [eye(3),[3;0;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body32.Joint = jnt1;
body32.Mass = 1; % first body (virtual)
body32.CenterOfMass = [1;0;0];
addBody(rst_tree,body32,'finger31');

body32 = rigidBody('finger33');
jointname = 'fixed';
jnt1 = rigidBodyJoint(jointname,'fixed');
W_T_base = [eye(3),[1;0;0];0,0,0,1]; % World to Base
setFixedTransform(jnt1,W_T_base);
body32.Joint = jnt1;
body32.Mass = 0; % first body (virtual)
body32.CenterOfMass = [0;0;0];
addBody(rst_tree,body32,'finger32');

%% add visual

addCollision(rst_tree.Base, "sphere", [0.2])
addCollision(rst_tree.Bodies{1}, "box", [0.4 0.5 0.4])
addCollision(rst_tree.Bodies{3}, "box", [0.2 0.5 0.8])
addCollision(rst_tree.Bodies{4}, "box", [0.1 0.2 0.3])


rst_tree.show([0,0,0,0,0,0,0,0]','Collisions','on','Frames','on')

H = massMatrix(rst_tree,[0,0,0,0,0,0,0,0]')
gravTorq = gravityTorque(rst_tree,[0,0,0,0,0,0,0,0]')

jacobian = geometricJacobian(rst_tree,[0,0,0,0,0,0,0,0]','finger33')

