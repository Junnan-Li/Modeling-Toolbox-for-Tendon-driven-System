% example of the Shadow Hand










clear all
close all
clc


%% load model


run examples\create_shadow_hand.m

close all

%% simulation settings
% n_q = 5 + 4 + 4 + 4 + 5 = 22
n_q = finger_index.nj;
% in_f_start = [1,6,10,14,18]; % start index of each finger
% in_f_stop = [5,9,13,17,22]; % stop index of each finger

% Finger_list = {finger_thumb,finger_index,finger_middle,finger_ring,finger_little}';

q_desired = [0;0.7;0.5;0.4];
qd_desired = zeros(n_q,1);
qdd_desired = zeros(n_q,1);
F_ext = zeros(6,n_q+2);

delta_t = 0.001; % unit s
end_time = 1*1.5; % unit s
tSpan = delta_t .* [0:end_time/delta_t];
step = length(tSpan); 
% F_ext = zeros(6,n_q+2);

% controller parameters
P = 0.004 * ones(n_q,1);
D = 0.0015 * ones(n_q,1);
gravity_com = 1; 


% if use mex function
mex = 0;
customized_solver = 0;
opts = odeset('InitialStep',1e-3);

%% Simulation
% initial variables
state = zeros(2*n_q,step); % [q;qd]
state_d = zeros(2*n_q,step); % [qd;qdd]

Tau = zeros(n_q,step);
Tau_error = zeros(n_q,step);
% f_tendon = zeros(n_t,step);
% f_tendon_pre = 20*ones(n_t,1);

Computation_Cost = zeros(step,1);

h = figure(2);
% finger_index.update_finger(q_0);
par_plot = finger_index.plot_parameter_init;
par_plot.axis_show = 0;
par_plot_des = par_plot;
par_plot_des.linecolor = 'c';
% hold off
axis equal
xlim([0,0.25])
ylim([-0.04,0.12])
zlim([-0.042,0.12])

for i = 1:step
    tic;
    
    state_i = state(:,i);
    q_i = state_i(1:n_q);
    qd_i = state_i(n_q+1:end);
    C_finger = finger_index.update_M_coupling(q_i);
     
    
    t_i = i*delta_t;
    state_i = [q_i;qd_i];
    
    q_theta_i = q_desired - q_i;
    qd_theta_i = qd_desired - qd_i;
    
    if gravity_com
        [~,~,~,G_fd] = finger_index.fordyn_ne_w_end(q_i, qd_i, zeros(n_q,1), F_ext, mex);        
        Tau_i = P.*q_theta_i + D.*qd_theta_i + G_fd;
    else
        Tau_i = P.*q_theta_i + D.*qd_theta_i;
    end
    
    if customized_solver
        [state_next,state_di] = rungekutta4(@(t_i,state_i)finger_index.fordyn(t_i ,state_i, Tau_i, F_ext, mex),delta_t,t_i,state_i);
%         [~,stated_i_out] = ode45(@(t_i,state_i) finger_r.fordyn(t_i ,state_i, Tau_i, F_ext, mex), [0 delta_t], state_i, opts);
        state(:,i+1) =  state_next;
        state_d(:,i) = state_di;
    else
        [qdd_i,M_fd,C_fd,G_fd] = finger_index.fordyn_ne_w_end(q_i, qd_i, Tau_i, F_ext, mex);
        state_d(:,i) = [qd_i;qdd_i];
        state(:,i+1) =  state(:,i)+delta_t.*state_d(:,i);
    end
    
%     [q_i_sat, qd_i_sat, qdd_i_sat] = finger_index.check_joints_limits(state(1:n_q,i+1), state(n_q+1:end,i+1), state_d(n_q+1:end,i));
%     state_d(:,i) = [qd_i_sat;qdd_i_sat];
%     state(:,i+1) = [q_i_sat;qd_i_sat];
    
    Tau(:,i) = Tau_i;
    [f_tendon_i, tau_real] = tda_s1(Tau_i,C_finger,finger_index.limits_ft(:,1),finger_index.limits_ft(:,2),1000,1e-4);
    Tau_error(:,i) = Tau_i-tau_real;
    f_tendon(:,i) = f_tendon_i;
    
    % animation
    cla
    finger_thumb.plot_finger(par_plot);
    finger_middle.update_finger([0,0,0,0])
    finger_middle.plot_finger(par_plot);
    finger_ring.update_finger([0,0,0,0])
    finger_ring.plot_finger(par_plot);
    finger_little.plot_finger(par_plot);
    finger_index.update_finger(q_i);
    finger_index.plot_finger(par_plot);
    finger_index.update_finger(q_desired);
    finger_index.plot_finger(par_plot_des);
    drawnow

%     export as gif
    if mod(i,10) == 0
        exportgraphics(gcf,'testAnimated.gif','Append',true);
    end 
    Computation_Cost(i) = toc;

end

fprintf('average computation cost each step: %f - %f \n', mean(Computation_Cost), delta_t);

%%  plot results
Color_joint = {'r','c','b','k'};

h = figure(1)
set(0,'defaultfigurecolor','w')
set(groot,'defaulttextinterpreter','none');
set(h,'units','normalized','outerposition',[0 0 1 1])
set(h, 'Units', 'centimeters')
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)]);%
set(h, 'Units', 'centimeters')
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
subplot(5,1,1)
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

subplot(5,1,2)
F_3 = plot(tSpan,state(n_q+1:end,1:end-1)); % Joint velocity
for i = 1:4
    set(F_3(i), 'color', Color_joint{i});
end
title('Joint velocity')
xlabel('Time (s)')
ylabel('velocity (rad/s)')

subplot(5,1,3)
plot(tSpan,Tau) % Joint torque
title('Joint torque')
xlabel('Time (s)')
ylabel('Torque (Nm)')

subplot(5,1,4)
plot(tSpan,f_tendon)
title('Tendon force')
xlabel('Time (s)')
ylabel('Tendon force (N)')


subplot(5,1,5)
plot(tSpan,Tau_error)
title('Torque error (TDA) (Nm)')
xlabel('Time (s)')
ylabel('Torque (Nm)')


