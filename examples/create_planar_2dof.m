%   example script for creating an example of finger

%% create finger with tendons

clear all 
close all
clc

% define a finger
robot_dimension = [4,2]; % in meter

robot_2dof = Finger('Index', 'type','R_RR', 'l_links',robot_dimension); 

mdh_default_struct = robot_2dof.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1); % alpha,a,theta,d
%% set states
% set base position and orientation
robot_2dof.w_p_base = 4*zeros(3,1);
robot_2dof.w_R_base = euler2R_XYZ([-pi/2;0;0]);

% init joint configurations
q_0 = zeros(2,1);

% udpate finger with given joint configurations
robot_2dof.update_finger(q_0);

% load rst model from finger class
robot_2dof.update_rst_model;
rst_model_r = robot_2dof.rst_model;

figure(1)
rst_model_r.show(q_0);
hold on
par_plot = robot_2dof.plot_parameter_init;
robot_2dof.plot_finger(par_plot);


% return
%% set dynamic parameters
% link index:
%   1: PP
%   2: MP
%   3: DP

robot_2dof.list_links(1).set_mass(1); % in kg
robot_2dof.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

robot_2dof.list_links(2).set_mass(3); % in kg
robot_2dof.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 



% update dynamic parameters
robot_2dof.update_finger_par_dyn;
% finger_r.update_finger(q_0);

% Tau = robot_2dof.invdyn_ne_w_end(q_0, zeros(2,1), zeros(2,1), zeros(6,1))

return

%% forward dynamic and animation

sim_time = 10; % s
dt = 1e-2; % s
step = sim_time/dt;


n_q = robot_2dof.nj;
q0 = [0,0]';
qd0 = [0,0]';

state = zeros(2*n_q,step); % [q;qd]
state(:,1) = [q0;qd0];
state_d = zeros(2*n_q,step);
T = zeros(step,1);

Tau = zeros(n_q,step);
F_ext = zeros(6,n_q+2);
mex = 0;

h = figure(2)
robot_2dof.update_finger(q0);
robot_2dof.plot_finger(par_plot);
% hold off
axis equal
xlim([-7,7])
ylim([-7,7])
zlim([-7,1])
view([15 10 5])
T = zeros(step,1);
for i = 1:step
    tic;
    
    state_i = state(:,i);
    q_i = state_i(1:n_q);
    qd_i = state_i(n_q+1:end);
%     q_theta_i = q_desired - q_i;
%     qd_theta_i = qd_desired - qd_i;
%     

    Tau_i = Tau(:,i);
    
    [qdd_i,M_fd,C_fd,G_fd] = robot_2dof.fordyn_ne_w_end(q_i, qd_i, Tau_i, F_ext, mex);
    
    %     stated_i = rungekutta4(@(t_i,state_i)finger_r.fordyn(t_i ,state_i, Tau_i, F_ext),delta_t,t_i,state_i);
    %     state(:,i+1) = delta_t.*stated_i + state_i;
    %     state_d(:,i) = stated_i;
%     
    state_d(1:n_q,i) = qd_i;
    state_d(n_q+1:end,i) = qdd_i;
    
    qd_i_next = qd_i + dt * qdd_i/2;
    q_i_next = q_i + dt * qd_i_next/2;

    state(:,i+1) =  [q_i_next;qd_i_next];
    T(i+1) = T(i)+dt;
    
    % update plot 
    cla
    robot_2dof.update_finger(q_i_next);
    robot_2dof.plot_finger(par_plot);
    drawnow
    
end









