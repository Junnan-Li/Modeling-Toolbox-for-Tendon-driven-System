% test script: inverse dynamic of serial robots with revolute joints 
% comparing results to the robot system toolbox of MATLAB
% 
% random joint states, X_base, external force, mass, inertia, com, 
% 
% Note: this test is only for fixed base due to the rst model


clear all
close all
clc

%% setting
show_robot = 0;
show_time_cost = 0;
% number of joints
n_q = 5; 

%% random configurations

% random robot states
q = pi*rand(n_q,1);
qD = 5*rand(n_q,1);
qDD = 5*rand(n_q,1);
% gravity acceleration
g = [0 0 -9.81]';

% random robot link mass
% Mass = [0;rand(n_q,1)]; % first term is the base mass
Mass = rand(n_q+1,1); % first term is the base mass

% random base position
X_base = rand(6,1);
XD_base = zeros(6,1);
XDD_base = zeros(6,1);

% random external force/moment 
F_ext_ne = rand(6,1);
F_ext_ne_all_fext = [rand(6,n_q+1),F_ext_ne];

% random link inertia of robot with respect to the Center of Mass for ne
% method
I_ne = zeros(6,n_q+1); % [xx yy zz yz xz xy]
I_rst = zeros(6,n_q+1); % [xx yy zz yz xz xy]

% assuming the base frame has the same position with the frame of link 1
CoM_ne = zeros(3,n_q+1);
CoM_rst = zeros(3,n_q+1);
for i = 1:n_q
    CoM_ne(:,i+1) = rand(3,1);
    CoM_rst(:,i+1) = CoM_ne(:,i+1);
    % MATLAB robotics system toolbox need Inertia with respect to the frame
    % origin 
    I_ne(:,i+1) = [rand(3,1);0;0;0]; 
    I_rst(:,i+1) = I_ne(:,i+1) + ...
        Mass(i+1)*[CoM_rst(2,i+1).^2+CoM_rst(3,i+1).^2;CoM_rst(1,i+1).^2+CoM_rst(3,i+1).^2;CoM_rst(1,i+1).^2+CoM_rst(2,i+1).^2;...
        -CoM_rst(2,i+1)*CoM_rst(3,i+1);-CoM_rst(1,i+1)*CoM_rst(3,i+1);-CoM_rst(1,i+1)*CoM_rst(2,i+1)];
end

% random mdh parameters
% for ne dunction: [alpha,a,theta,d] 
% for rst: [a,alpha,d,theta]
mdh_base = zeros(1,4);
mdh_ne_link = rand(n_q,4);
mdh_ne_link(:,3) = zeros(n_q,1); % assuming all theta are 0 (changing by q_i)
mdh_ne = [mdh_base;mdh_ne_link];
mdh_rst = [mdh_ne(:,2),mdh_ne(:,1),mdh_ne(:,4),mdh_ne(:,3)];% change the sequence of the mdh format for rst tree

%%  Robotic system toolbox

robot = rigidBodyTree;
robot.DataFormat = 'column';
robot.Gravity = g;
last_link_name = 'base';

% add links
for i = 1:n_q
    link_i_name = char("link"+num2str(i));
    bodyi = rigidBody(link_i_name);
    joint_i_name = char("jnt"+num2str(i));
    jnt_i = rigidBodyJoint(joint_i_name,'revolute');
    if i == 1
        W_T_base = [euler2R_XYZ(X_base(4:6)),X_base(1:3); 0 0 0 1];
        setFixedTransform(jnt_i,W_T_base); % frame 1 position from X_base
    else
        setFixedTransform(jnt_i,mdh_rst(i,:),'mdh');
    end
    bodyi.Joint = jnt_i;
    bodyi.Mass = Mass(i+1);
    bodyi.Inertia = I_rst(:,i+1);
    bodyi.CenterOfMass = CoM_rst(:,i+1);
    addBody(robot,bodyi,last_link_name)
    
    last_link_name = link_i_name;
end

body_end = rigidBody('endeff');
jnt_end = rigidBodyJoint('jnt_end','fixed');
setFixedTransform(jnt_end,mdh_rst(end,:),'mdh');
body_end.Joint = jnt_end;
body_end.Mass = 0;
body_end.Inertia = [0,0,0,0,0,0];
body_end.CenterOfMass = [0,0,0];
addBody(robot,body_end,last_link_name)


F_ext_rst = zeros(6,n_q+1);
for i = 1:n_q
    link_i_name = char("link"+num2str(i));
    % transfer the external force exerting on the
    transform = getTransform(robot,q,link_i_name);
    W_R_i = transform(1:3,1:3);
    F_ext_rst_i = externalForce(robot,link_i_name,[W_R_i'*F_ext_ne_all_fext(4:6,i+1);W_R_i'*F_ext_ne_all_fext(1:3,i+1)],q);
    % calculate the inverse dynamics
    F_ext_rst = F_ext_rst + F_ext_rst_i;
end

transform = getTransform(robot,q,'endeff');
W_R_end = transform(1:3,1:3);
F_ext_rst_end = externalForce(robot,'endeff',[W_R_end'*F_ext_ne_all_fext(4:6,end);W_R_end'*F_ext_ne_all_fext(1:3,end)],q);
F_ext_rst = F_ext_rst + F_ext_rst_end;

Tau_rst_all_fext = inverseDynamics(robot, q, qD, qDD, F_ext_rst);
Tau_rst_end_fext = inverseDynamics(robot, q, qD, qDD, F_ext_rst_end);
if show_robot
    robot.show(q,'Collisions','on','Visuals','on');
end

%% Newton-Euler inverse dynamic all f_ext(ne method)
% invdyn_ne_mdh_all_fext.m
tic;
[Tau_ne_all_fext,F,W_T_allframe] = invdyn_ne_mdh_all_fext(q,qD,qDD,mdh_ne, Mass,...
    X_base, XD_base, XDD_base, F_ext_ne_all_fext, CoM_ne, I_ne, g);
t_stop = toc;
if show_time_cost
    fprintf('inverse dynamic computation cost: %f \n', t_stop);
end
% tic;
% [Tau_ne_all_fext,F,W_T_allframe] = invdyn_ne_mdh_all_fext_mex(q,qD,qDD,mdh_ne, Mass,...
%     X_base, XD_base, XDD_base, F_ext_ne_all_fext, CoM_ne, I_ne, g);
% t_stop = toc;
% if show_time_cost
%     fprintf('mex inverse dynamic computation cost: %f \n', t_stop);
% end

% Validation: Inverse dynamic Torque
T_all_fext_error = Tau_rst_all_fext - Tau_ne_all_fext;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(T_all_fext_error(:))) > 1e-8
    fprintf('Inverse dynamic test all f_ext (NE Torque): failed! \n')
else
    fprintf('Inverse dynamic test all f_ext (NE Torque): pass! \n')
end

%% Newton-Euler inverse dynamic only endeffector f_ext(ne method)
% invdyn_ne_mdh.m
tic;
[Tau_ne_end_fext,F,W_T_allframe] = invdyn_ne_mdh(q,qD,qDD,mdh_ne, Mass,...
    X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g);
t_stop = toc;
if show_time_cost
    fprintf('inverse dynamic computation cost: %f \n', t_stop);
end
% tic;
% [Tau_ne_end_fext,F,W_T_allframe] = invdyn_ne_mdh_mex(q,qD,qDD,mdh_ne, Mass,...
%              X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g);
% t_stop = toc;

[Tau_ne_all_fext,F,W_T_allframe] = invdyn_ne_mdh_all_fext(q,qD,qDD,mdh_ne, Mass,...
    X_base, XD_base, XDD_base, [zeros(6,n_q+1),F_ext_ne], CoM_ne, I_ne, g);


if show_time_cost
    fprintf('mex inverse dynamic computation cost: %f \n', t_stop);
end
% Validation: Inverse dynamic Torque
T_end_fext_error = Tau_rst_end_fext - Tau_ne_end_fext;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(T_end_fext_error(:))) > 1e-8
    fprintf('Inverse dynamic test endedffector f_ext (NE Torque): failed! \n')
else
    fprintf('Inverse dynamic test endedffector f_ext (NE Torque): pass! \n')
end

%% Newton-Euler inverse dynamic gravity 
% invdyn_ne_mdh_all_fext_gravity.m

Tau_g_rst = gravityTorque(robot,q);

Tau_g_ne = invdyn_ne_mdh_all_fext_gravity(q,mdh_ne, Mass, X_base, CoM_ne, g);

% Validation: Inverse dynamic Torque
Tau_g_error = Tau_g_rst - Tau_g_ne;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(Tau_g_error(:))) > 1e-8
    fprintf('Inverse dynamic geavity test (NE Torque): failed! \n')
else
    fprintf('Inverse dynamic geavity test (NE Torque): pass! \n')
end

return
%% Forward dynamic 
% given q, qD, tau  --> qDD

Tau = Tau_ne_all_fext;

J_geom = geometricJacobian(robot,q,'endeff');

%%%%%%%%% rst model

M_rst = massMatrix(robot,q);
C_rst = velocityProduct(robot,q,qD);
G_rst = gravityTorque(robot,q);
qDD_rst = forwardDynamics(robot,q,qD,Tau,F_ext_rst);

%%%%%%%% Newton Euler Method 
tic;
[qDD_ne_est,M_ne_fd,C_ne_fd,G_ne_fd] = fordyn_ne_mdh(q,qD,Tau,mdh_ne, Mass,...
             X_base, XD_base, XDD_base, F_ext_ne_all_fext, CoM_ne, I_ne, g, 0);
t_stop = toc;
if show_time_cost
    fprintf('forward dynamic computation cost: %f \n', t_stop);
end
tic;
[qDD_ne_est,M_ne_fd,C_ne_fd,G_ne_fd] = fordyn_ne_mdh(q,qD,Tau,mdh_ne, Mass,...
    X_base, XD_base, XDD_base, F_ext_ne_all_fext, CoM_ne, I_ne, g, 0);
t_stop = toc;
if show_time_cost
    fprintf('mex forward dynamic computation cost: %f \n', t_stop);
end

% Geometry Jacobian of the endeffector
J_rst = [J_geom(4:6,:);J_geom(1:3,:)];
% use Jacobian_geom_mdh.m to calculate geometric Jacobian
w_R_b = euler2R_XYZ(X_base(4:6));
b_J_ne = Jacobian_geom_mdh(mdh_ne,q);
J_ne = blkdiag(w_R_b,w_R_b)*b_J_ne;



%% Validation

% Inverse dynamic Mass matrix
M_error = M_rst - M_ne_fd;
if max(abs(M_error(:))) > 1e-8
    fprintf('Inverse dynamic test (NE Mass Matrix): failed! \n')
else
    fprintf('Inverse dynamic test (NE Mass Matrix): pass! \n')
end

% Inverse dynamic Velocity-dependnet torque 
C_error = C_rst - C_ne_fd;
if max(abs(C_error(:))) > 1e-8
    fprintf('Inverse dynamic test (NE C torque): failed! \n')
else
    fprintf('Inverse dynamic test (NE C torque): pass! \n')
end

% Inverse dynamic gravity term 
G_error = G_rst - G_ne_fd;
if max(abs(G_error(:))) > 1e-8
    fprintf('Inverse dynamic test (NE gravity): failed! \n')
else
    fprintf('Inverse dynamic test (NE gravity): pass! \n')
end



% Goemetric Jacobian J
J_error = J_rst - J_ne;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(J_error(:))) > 1e-8
    fprintf('Goemetric Jacobian test (NE qDD): failed! \n')
else
    fprintf('Goemetric Jacobian test (NE qDD): pass! \n')
end

% forward dynamic qDD
qDD_error = qDD_rst - qDD_ne_est;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(qDD_error(:))) > 1e-8
    fprintf('Forward dynamic test (NE qDD): failed! \n')
else
    fprintf('Forward dynamic test (NE qDD): pass! \n')
end




