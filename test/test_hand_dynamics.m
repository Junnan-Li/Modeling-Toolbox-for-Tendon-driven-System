% test Hand and associated dynamics


close all
clear all
clc


%% 

run examples\create_hand_vp_mus.m
plot_par = hand.plot_parameter_init;
plot_par.axis_len = 0.5;
hand.update_hand_par_dyn
hand_rst = hand.update_rst_model;

figure(1)
hand.plot_hand(plot_par)
hand.plot_hand_com(plot_par)
axis equal
hand_rst.show(q,'Frames','on');

%% ID test

q_init = zeros(hand.nj,1);
test_failed = 0;

for test_iter = 1:10
    
    hand.set_base(rand(3,1),euler2R_XYZ(rand(3,1)));
    hand.update_hand(q_init);
%     hand
    hand_rst = hand.update_rst_model;
    q = rand(hand.nj,1);
    qd = rand(hand.nj,1);
    qdd = rand(hand.nj,1);

    hand.update_hand(q);

%     % plot 
    figure(2)
    hand.plot_hand(plot_par)
    hand.plot_hand_com
    axis equal
    hand_rst.show(q,'Frames','on');

    tau_rst = hand_rst.inverseDynamics(q,qd,qdd); % gravity torque
    Tau = hand.invdyn_ne_hand_w_end(q,qd,qdd);

    error_tau = tau_rst-Tau;
    if max(abs(error_tau(:))) > 1e-10
        test_failed = 1;
        break
    end
end
if test_failed == 1
    fprintf('Test (ID): failed! \n')
else
    fprintf('Test (ID): pass! \n')
end

% return
%% Mass matrix, coriolis and gravity
q_init = zeros(hand.nj,1);
test_failed = 0;
for test_iter = 1:10
    % 
    hand.set_base(rand(3,1),euler2R_XYZ(rand(3,1)));
    hand.update_hand(q_init);
%     hand
    hand_rst = hand.update_rst_model;
    q = rand(hand.nj,1);
    qd = rand(hand.nj,1);
    tau = rand(hand.nj,1);
    hand.update_hand(q);

    % plot 
    figure(2)
    hand.plot_hand(plot_par)
    hand.plot_hand_com
    axis equal
    hand_rst.show(q,'Frames','on');

    M_rst = hand_rst.massMatrix(q);
    G_rst = hand_rst.gravityTorque(q);
    C_rst = hand_rst.velocityProduct(q,qd);
    qdd_rst = hand_rst.forwardDynamics(q,qd,tau);


    [qDD,M,C,G] = hand.fordyn_ne_hand_w_end( q,qd, tau);

    error_M = M_rst-M;
    error_C = C_rst-C;
    error_G = G_rst-G;
    error_qdd = qdd_rst-qDD;
    error_vec = [error_C;error_G;error_qdd];
    if max(abs(error_M(:))) > 1e-10 || max(abs(error_vec(:))) > 1e-10
        test_failed = 1;
    end
end
if test_failed == 1
    fprintf('Test (FD): failed! \n')
else
    fprintf('Test (FD): pass! \n')
end

%% forward dynamics


q_init = zeros(hand.nj,1);
hand.update_hand(q_init);
hand.init_state;
f_mus = zeros(hand.nmus,1);

sim_t_end = 10; 
sim_step = 0.005;

data = struct();
data.time = [];
data.state = [];
for i = 0:sim_step:sim_t_end
    data.time = [data.time;i];
    ma = hand.get_Muscle_Momentarm_1st_c;
    tau_mus = ma*f_mus;
    state_i = hand.fordyn_step(sim_step,tau_mus);
    data.state = [data.state;reshape(state_i,1,length(state_i))]; 
end

%%
figure(10)
for i = 1:length(data.time)
    hand.update_hand(data.state(i,1:hand.nj));
    hand.plot_hand(plot_par);
    drawnow
    hold off
end


