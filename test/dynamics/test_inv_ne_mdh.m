% test script: inverse dynamic of serial robots with revolute joints 
% comparing results to the robot system toolbox of MATLAB



%% setting
show_robot = 0;

%% random configurations
% number of joints
n_q = 5; 
% random robot states
q = pi*rand(n_q,1);
qD = 5*rand(n_q,1);
qDD = 5*rand(n_q,1);
% gravity acceleration
g = [0 0 -9.81]';

% random robot link mass
Mass = [0;rand(n_q,1)]; % first term is the base mass

X_base = zeros(6,1);
XD_base = zeros(6,1);
XDD_base = zeros(6,1);

% random external force/moment 
F_ext_ne = 5*rand(6,1);

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


%%  Robot system toolbox

robot = rigidBodyTree;
robot.DataFormat = 'column';
robot.Gravity = g;
last_link_name = 'base';
for i = 1:n_q
    link_i_name = char("link"+num2str(i));
    bodyi = rigidBody(link_i_name);
    joint_i_name = char("jnt"+num2str(i));
    jnt_i = rigidBodyJoint(joint_i_name,'revolute');
    setFixedTransform(jnt_i,mdh_rst(i,:),'mdh');
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

% M_rst = massMatrix(robot,q);
% G_rst = gravityTorque(robot,q);

% transfer the external force exerting on the 
transform = getTransform(robot,q,'endeff');
W_R_end = transform(1:3,1:3);
F_ext_rst = externalForce(robot,'endeff',[W_R_end'*F_ext_ne(4:6);W_R_end'*F_ext_ne(1:3)],q);

% calculate the inverse dynamics
Tau_rst = inverseDynamics(robot, q, qD, qDD, F_ext_rst);

if show_robot 
    robot.show(q,'Collisions','on','Visuals','on');
end

%% Newton-Euler inverse dynamic (ne method)

[Tau_ne,F,W_T_allframe] = invdyn_ne_mdh(q,qD,qDD,mdh_ne, Mass,...
             X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g);

%% Validation

% Torque
T_error = Tau_rst - Tau_ne;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(T_error(:))) > 1e-8
    fprintf('Newton-Euler: Torque test: failed! \n')
else
    fprintf('Newton-Euler:Torque test: pass! \n')
end









