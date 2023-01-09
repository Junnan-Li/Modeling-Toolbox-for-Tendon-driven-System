%   two links serial robots 
%   inverse dynamics of Lagrangian formulation






clear all
close all
clc


%% robot parameters 

m1 = 2;
m2 = 1;

l1 = 2;
l2 = 1;

la1 = 1;
la2 = 0.5;

% Inertia regarding center of mass
Izz1 = 2.1; % 
Izz2 = 1.1;

g = [0 -9.81 0]';

%% random configurations
q = pi*rand(2,1);
qD = 5*rand(2,1);
qDD = 5*rand(2,1);

%%  Robot system toolbox

% mdh parameters
mdh_par = [0,0,0,0;...
    l1,0,0,0;...
    l2,0,0,0];
q = [0,0]';
       
robot = rigidBodyTree;
robot.DataFormat = 'column';
robot.Gravity = g;

body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1,mdh_par(1,:),'mdh');
body1.Joint = jnt1;
body1.Mass = m1;
body1.Inertia = [0,0,Izz1+m1*la1^2,0,0,0];
body1.CenterOfMass = [la1,0,0];
addBody(robot,body1,'base')

body2 = rigidBody('link2');
jnt2 = rigidBodyJoint('jnt2','revolute');
setFixedTransform(jnt2,mdh_par(2,:),'mdh');
body2.Joint = jnt2;
body2.Mass = m2;
body2.Inertia = [0,0,Izz2+m2*la2^2,0,0,0];
body2.CenterOfMass = [la2,0,0];
addBody(robot,body2,'link1')

body3 = rigidBody('endeff');
jnt3 = rigidBodyJoint('jnt3','fixed');
setFixedTransform(jnt3,mdh_par(3,:),'mdh');
body3.Joint = jnt3;
body3.Mass = 0;
body3.Inertia = [0,0,0,0,0,0];
body3.CenterOfMass = [0,0,0];
addBody(robot,body3,'link2')

M_rst = massMatrix(robot,q);
G_rst = gravityTorque(robot,q);
T_rst = inverseDynamics(robot,q,qD,qDD)
% centerOfMass(robot)
robot.show(q,'Collisions','on','Visuals','on');

%% Lagrandian method

% each terms of matrix
M_lag = MassMatrix_twolinks(m1,m2,Izz1,Izz2,l1,l2,la1,la2, q);
C_lag = CMatrix_twolinks(m1,m2,l1,l2,la1,la2, q, qD);
G_lag = GMatrix_twolinks(m1,m2,l1,l2,la1,la2, q,-g(2));
T_lag = M_lag*qDD + C_lag*qD + G_lag;


%% Newton-euler method
% parameters
Mass = [0;m1;m2];
X_base = zeros(6,1);
XD_base = zeros(6,1);
XDD_base = zeros(6,1);
F_ext = zeros(6,1);
CoM = [zeros(3,1),[la1,0,0]',[la2,0,0]'];
I = [zeros(6,1),[0,0,Izz1,0,0,0]',[0,0,Izz2,0,0,0]'];
mdh = [0,0,0,0;...
       0,l1,0,0;...
       0,l2,0,0];
   
[Tau_ne,F] = invdyn_ne_mdh(q,qD,qDD,mdh, Mass, X_base, XD_base, XDD_base, F_ext, CoM, I, g);


%% Validation

% Massmatrix
M_e_lag = M_rst - M_lag;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(M_e_lag(:))) > 1e-8
    fprintf('Lagrangian: Mass matrix test: failed! \n')
else
    fprintf('Lagrangian: Mass matrix test: pass! \n')
end

% Gravity torque
G_e_lag = G_rst - G_lag;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(G_e_lag(:))) > 1e-8
    fprintf('Lagrangian: Gravity torque test: failed! \n')
else
    fprintf('Lagrangian: Gravity torque test: pass! \n')
end

% Torque
T_e_lag = T_rst - T_lag;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(T_e_lag(:))) > 1e-8
    fprintf('Lagrangian: Torque test: failed! \n')
else
    fprintf('Lagrangian: Torque test: pass! \n')
end

% Torque
T_e_ne = T_rst - Tau_ne;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(T_e_ne(:))) > 1e-8
    fprintf('Newton-Euler: Torque test: failed! \n')
else
    fprintf('Newton-Euler:Torque test: pass! \n')
end

