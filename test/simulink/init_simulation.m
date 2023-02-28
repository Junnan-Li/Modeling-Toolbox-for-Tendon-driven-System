






run create_hand.m
n_q = finger_r.nj;
% finger object: finger_r
q_init = [0;0;0;0]*pi/180;
qD_init = [0;0;0;0]*pi/180;

F_ext = zeros(6,n_q+2);



mdh_ne = mdh_struct_to_matrix(finger_r.mdh_ori, 1);
mdh_ne(1:finger_r.nj,3) = mdh_ne(1:finger_r.nj,3); 

q_desired = [1,1,1,1]';

sim('./test/simulink/finger_simulink.slx');

return

%% simulation settings
n_q = 4;
q_desired = [0,pi/3,pi/4,pi/4]';
qd_desired = zeros(4,1);
qdd_desired = zeros(4,1);

delta_t = 0.001; % unit s
end_time = 1*60; % unit s
tSpan = 0:delta_t:end_time;
step = length(tSpan); 
initialState = [q_init;qD_init]; % [q;qd]
F_ext = zeros(6,n_q+2);
% controller parameters
P = zeros(4,1);%[2;70;40;5];
D = zeros(4,1);%[2;1;1;5];

% if use mex function
mex = 1;
gravity_com = 0; % if using gravity compensation
%% simulation

state = zeros(2*n_q,step);
state(:,1) = initialState;
state_d = zeros(2*n_q,step);

Tau = zeros(n_q,step);


T = zeros(step,1);
for i = 1:step
    tic;
    
    state_i = state(:,i);
    q_i = state_i(1:n_q);
    qd_i = state_i(n_q+1:end);
%     t_i = i*delta_t;
    %     state_i = [q_i;qd_i];

    q_theta_i = q_desired - q_i;
    qd_theta_i = qd_desired - qd_i;
    
    if gravity_com
        [~,~,~,G_fd] = finger_r.fordyn_ne_w_end(q_i, qd_i, zeros(n_q,1), F_ext);
        Tau_i = P.*q_theta_i + D.*qd_theta_i + G_fd;
    else
        Tau_i = P.*q_theta_i + D.*qd_theta_i;
    end
        
    
    [qdd_i,M_fd,C_fd,G_fd] = finger_r.fordyn_ne_w_end(q_i, qd_i, Tau_i, F_ext, mex);
    
    %     stated_i = rungekutta4(@(t_i,state_i)finger_r.fordyn(t_i ,state_i, Tau_i, F_ext),delta_t,t_i,state_i);
    %     state(:,i+1) = delta_t.*stated_i + state_i;
    %     state_d(:,i) = stated_i;
    
    state_d(1:n_q,i) = qd_i;
    state_d(n_q+1:end,i) = qdd_i;

    state(:,i+1) =  state(:,i)+delta_t.*state_d(:,i);
    Tau(:,i) = Tau_i;
    T(i) = toc;
end

fprintf('average computation cost each step: %f \n', mean(T));

%% results
figure(2)
subplot(2,1,1)
plot(tSpan,state(1:n_q,1:end-1)) % Joint position

% hold on
% plot(tSpan,q_desired.*ones(n_q,length(tSpan)),'--') % Joint setpoint
title(' Joint Position')
xlabel('Time (s)')
ylabel('Position (rad)')
legend('1','2','3','4')
subplot(2,1,2)
plot(tSpan,Tau) % Joint velocity
title('Joint torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')

