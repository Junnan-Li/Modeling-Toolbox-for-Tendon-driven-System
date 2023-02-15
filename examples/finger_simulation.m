% Simulation of a Finger





clear all 
close all
clc



% define a finger
finger_dimension = [0.005,0.003,0.002]; % in meter

finger_r = Finger('Index', 'RRRR', finger_dimension); 
%% set states
% set base position and orientation
finger_r.w_p_base = 4*zeros(3,1);
finger_r.w_R_base = euler2R_XYZ(zeros(1,3));

% init joint configurations
q_0 = [0;0.1;0.1;0.1];

% udpate finger with given joint configurations
finger_r.update_finger(q_0);
% load rst model from finger class
rst_model = finger_r.rst_model;
% rst_model.show(q_0)

%% set dynamic parameters
% link index:
%   1: PP
%   2: MP
%   3: DP

finger_r.list_links(1).set_mass(0.05); % in kg
finger_r.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(2).set_mass(0.03); % in kg
finger_r.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(3).set_mass(0.03); % in kg
finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

% update dynamic parameters
finger_r.update_finger_par_dyn;
% finger_r.update_finger(q_0);

%% add tendons

if finger_r.nt == 0
    finger_r.add_tendon('Flex_1', [1,1,1,1]);
    finger_r.add_tendon('Flex_2', [1,1,1,0]);
    finger_r.add_tendon('Flex_3', [1,1,0,0]);
    finger_r.add_tendon('Flex_4', [1,0,0,0]);
    finger_r.add_tendon('Ext_1', [-1,-1,-1,-1]);
    finger_r.add_tendon('Ext_2', [-1,-1,-1,0]);
    finger_r.add_tendon('Ext_3', [-1,-1,0,0]);
    finger_r.add_tendon('Ext_4', [-1,0,0,0]);
end

finger_r.set_tendon_par_MA_poly3(1,1,[0,0,0.01,0.03]);
finger_r.set_tendon_par_MA_poly3(3,1,[0,0,0.01,0.3]);
finger_r.set_tendon_par_MA_poly3(3,2,[0,0,0.01,0.035]);
finger_r.set_tendon_par_MA_poly3(4,1,[0,0,0.001,0.02]);

finger_r.update_finger(q_0);
finger_r.update_M_coupling(q_0);
finger_r.M_coupling


% set joint limits
finger_r.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints

finger_r.update_joints_info;

%% simulation settings
n_q = finger_r.nj;
n_t = finger_r.nt;
% init state of q
q_0 = [0;0.15;0.2;0.2];
finger_r.update_finger(q_0);

q_desired = [0.1,1,1,pi/2]';
qd_desired = zeros(4,1);
qdd_desired = zeros(4,1);

delta_t = 0.001; % unit s
end_time = 1*5; % unit s
tSpan = delta_t .* [0:end_time/delta_t];
step = length(tSpan); 
initialState = [q_0;zeros(4,1)]; % [q;qd]
F_ext = zeros(6,1);

% controller parameters
P = [1;1;1;1];
D = [1;1;1;1];
gravity_com = 1; 


% if use mex function
mex = 1;
% 
customized_solver = 1;
opts = odeset('InitialStep',1e-3);

%% Simulation
% initial variables
state = zeros(2*n_q,step); % [q;qd]
state(:,1) = initialState;
state_d = zeros(2*n_q,step); % [qd;qdd]

Tau = zeros(n_q,step);
Tau_error = zeros(n_q,step);
f_tendon = zeros(n_t,step);
% f_tendon_pre = 20*ones(n_t,1);

Computation_Cost = zeros(step,1);

for i = 1:step
    tic;
    
    state_i = state(:,i);
    q_i = state_i(1:n_q);
    qd_i = state_i(n_q+1:end);
    C_finger = finger_r.update_M_coupling(q_i);
     
    
    t_i = i*delta_t;
    state_i = [q_i;qd_i];
    
    q_theta_i = q_desired - q_i;
    qd_theta_i = qd_desired - qd_i;
    
    if gravity_com
        [~,~,~,G_fd] = finger_r.fordyn_ne_w_end(q_i, qd_i, zeros(n_q,1), F_ext, mex);        
        Tau_i = P.*q_theta_i + D.*qd_theta_i + G_fd;
    else
        Tau_i = P.*q_theta_i + D.*qd_theta_i;
    end
    
    if customized_solver
        [state_next,state_di] = rungekutta4(@(t_i,state_i)finger_r.fordyn(t_i ,state_i, Tau_i, F_ext, mex),delta_t,t_i,state_i);
%         [~,stated_i_out] = ode45(@(t_i,state_i) finger_r.fordyn(t_i ,state_i, Tau_i, F_ext, mex), [0 delta_t], state_i, opts);
        state(:,i+1) =  state_next;
        state_d(:,i) = state_di;
    else
        [qdd_i,M_fd,C_fd,G_fd] = finger_r.fordyn_ne_w_end(q_i, qd_i, Tau_i, F_ext, mex);
        state_d(:,i) = [qd_i;qdd_i];
        state(:,i+1) =  state(:,i)+delta_t.*state_d(:,i);
    end
    
    [q_i_sat, qd_i_sat, qdd_i_sat] = finger_r.check_joints_limits(state(1:n_q,i+1), state(n_q+1:end,i+1), state_d(n_q+1:end,i));
    state_d(:,i) = [qd_i_sat;qdd_i_sat];
    state(:,i+1) = [q_i_sat;qd_i_sat];
    
    Tau(:,i) = Tau_i;
    [f_tendon_i, tau_real] = tda_s1(Tau_i,C_finger,finger_r.limits_ft(:,1),finger_r.limits_ft(:,2),1000,1e-4);
    Tau_error(:,i) = Tau_i-tau_real;
    f_tendon(:,i) = f_tendon_i;

    
    Computation_Cost(i) = toc;
end

fprintf('average computation cost each step: %f - %f \n', mean(Computation_Cost), delta_t);


%% Results
Color_joint = {'r','c','b','k'};

figure(1)
subplot(2,1,1)
F_1 = plot(tSpan,state(1:n_q,1:end-1)); % Joint position

hold on
F_2 = plot(tSpan,q_desired.*ones(n_q,length(tSpan)),'--'); % Joint setpoint

for i = 1:4
    set(F_1(i), 'color', Color_joint{i});
    set(F_2(i), 'color', Color_joint{i});
end
title('Joint Position')
xlabel('Time (s)')
ylabel('Position (rad)')
legend('simulated','desired')

subplot(2,1,2)
F_3 = plot(tSpan,state(n_q+1:end,1:end-1)); % Joint velocity
for i = 1:4
    set(F_3(i), 'color', Color_joint{i});
end
title('Joint velocity')
xlabel('Time (s)')
ylabel('velocity (rad/s)')


% plot tendon force
figure(2)
subplot(2,1,1)
plot(tSpan,Tau) % Joint torque
title('Joint torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')

subplot(2,1,2)
plot(tSpan,f_tendon) % Joint velocity
title('Tendon force')
xlabel('Time (s)')
ylabel('Tendon force (N)')
