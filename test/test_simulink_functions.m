% test non-class function used for simulink computation
% 
%           
% 


clear all
close all
clc

%% generate hand 
hand = create_hand_random('hand_function_test', [2,2,4,4] );

hand.create_sim_functions_hand
%% transformation matrix of each links
%  sim_w_T_all_links_from_q.m
test_failed = 0;
t1 = 0;
t2 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    hand.update_hand(q);
    w_T_links_all_class = zeros(4,4,hand.nj);
    for i = 1:hand.nl
        w_T_links_all_class(:,:,i) = hand.list_links(i).get_w_T_Link_inhand;
    end
    tic
    w_T_all_links_cla = hand.get_w_T_links_inhand;
    t1 = t1 + toc;
    hand.update_sim_par;
    tic
    w_T_all_links = sim_w_T_all_links_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);
    t2 = t2 + toc;

    T_error1 = w_T_links_all_class-w_T_all_links_cla;
    T_error2 = w_T_links_all_class-w_T_all_links;
    T_error = [T_error1;T_error2];
    
    if max(abs(T_error(:))) > 1e-8 
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (sim_w_T_all_links_from_q.m): FAILED!!! \n')
else
    fprintf('Test (sim_w_T_all_links_from_q.m): PASS!!! \n')
end

fprintf('Time cost: \n')
fprintf('object function:            %f \n',t1/iter)
fprintf('sim function:               %f \n',t2/iter)
fprintf('------------------------ \n')
%% Transformation matrix for all frames
% test sim_w_T_all_frames_from_q.m with hand.get_w_T_all
test_failed = 0;
% t1 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    hand.update_hand(q);
    w_T_all_frames = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);
    
    T = hand.get_w_T_all();
    w_T_links_inhand = hand.get_w_T_links_inhand;

    T_f_error = w_T_all_frames-T;
    if max(abs([T_f_error(:)])) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (sim_w_T_all_frames_from_q.m): FAILED!!! \n')
else
    fprintf('Test (sim_w_T_all_frames_from_q.m): PASS!!! \n')
end
fprintf('------------------------ \n')

%% Jacobian matrix 
% sim_w_Jacobian_geom_from_T_links.m
% compare to hand.Jacobian_geom_w_point
% 
test_failed = 0;
t1 = 0;
t2 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    hand.update_hand(q);
    w_T_links_inhand = hand.get_w_T_links_inhand;
    link_index = randi(hand.nl);
    i_p_point = rand(3,1);
    w_p_point = T_p31(w_T_links_inhand(:,:,link_index), i_p_point);
    [~,J_all] = hand.Jacobian_geom_w_point(link_index,i_p_point);
    tic
    J = sim_w_Jacobian_geom_from_T_links(w_T_links_inhand,...
        hand.sim.n_links, hand.sim.q_index, link_index, w_p_point);   
    t1 = t1 + toc;
    tic
    J_sim = hand_function_test_sim_w_Jacobian_geom_from_T_links( ...
        w_T_links_inhand, link_index, w_p_point);
    t2 = t2 + toc;
    J_error = J_all - J;
    J_error2 = J_all - J_sim;
    if max(abs([J_error(:);J_error2(:)])) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (sim_w_Jacobian_geom_from_T_links.m): FAILED!!! \n')
else
    fprintf('Test (sim_w_Jacobian_geom_from_T_links.m): PASS!!! \n')
end
fprintf('Time cost: \n')
fprintf('sim_w_Jacobian_geom_from_T_links:            %f \n',t1/iter)
fprintf('sim_w_Jacobian_geom_from_T_links.t:          %f \n',t2/iter)
fprintf('------------------------ \n')
%% Inverse dynamic 
% test invdyn_ne_T.m with rst.inverseDynamics

test_failed = 0;
t1 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    qd = rand(hand.nj,1);
    qdd = rand(hand.nj,1);
    hand.update_hand(q);
    hand.update_sim_par;
    hand.update_hand_par_dyn;
    hand_rst = hand.update_rst_model;
    w_T_all_frames = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);

    X_base = zeros(6,1);
    X_base(1:3) = hand.w_p_base;
    X_base(4:6) = R2euler_XYZ(hand.w_R_base);
    XD_base = zeros(6,1);
    XDD_base = zeros(6,1);
    F_ext = zeros(6,hand.nf);
    tic
    tau_obj =  invdyn_ne_T(w_T_all_frames,qd,qdd, hand.sim.n_links, hand.sim.q_index, ...
        hand.sim.Mass, XD_base,XDD_base, F_ext, hand.sim.CoM, hand.sim.I, hand.sim.g);
    t1 = t1 + toc;
    tau_rst = hand_rst.inverseDynamics(q,qd,qdd);
    error_tau = max(abs(tau_obj - tau_rst));

    if max(abs(error_tau)) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (invdyn_ne_T.m): FAILED!!! \n')
else
    fprintf('Test (invdyn_ne_T.m): PASS!!! \n')
end
fprintf('Time cost: \n')
fprintf('invdyn_ne_T:                  %f \n',t1/iter)
fprintf('------------------------ \n')
%%
% test hand_function_test_sim_w_T_all_frames_from_q.m 
% with sim_w_T_all_frames_from_q.m
test_failed = 0;
% t1 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    w_T_all_frames = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);
    w_T_all_frames_t = hand_function_test_sim_w_T_all_frames_from_q(q);
    
    T_f_error = w_T_all_frames-w_T_all_frames_t;
    if max(abs([T_f_error(:)])) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (sim transformation function): FAILED!!! \n')
else
    fprintf('Test (sim transformation function): PASS!!! \n')
end
fprintf('------------------------ \n')

%% sim func inverse dynamic 

test_failed = 0;
t1 = 0;
t2 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    qd = rand(hand.nj,1);
    qdd = rand(hand.nj,1);
    hand.update_hand(q);
    hand.update_sim_par;
    hand.update_hand_par_dyn;
    hand_rst = hand.update_rst_model;
    w_T_all_frames = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);

    X_base = zeros(6,1);
    X_base(1:3) = hand.w_p_base;
    X_base(4:6) = R2euler_XYZ(hand.w_R_base);
    XD_base = zeros(6,1);
    XDD_base = zeros(6,1);
    F_ext = zeros(6,hand.nf);
    tic
    tau_obj = invdyn_ne_T(w_T_all_frames,qd,qdd, hand.sim.n_links, hand.sim.q_index, ...
        hand.sim.Mass, XD_base,XDD_base, F_ext, hand.sim.CoM, hand.sim.I, hand.sim.g);
    t1 = t1 + toc;
    tic
    tau_sim = hand_function_test_invdyn_ne_T(w_T_all_frames,qd,qdd, XD_base,XDD_base, F_ext);
    t2 = t2 + toc;
    tau_rst = hand_rst.inverseDynamics(q,qd,qdd);
    error_tau = max(abs(tau_obj - tau_sim));
    error_tau2 = max(abs(tau_obj - tau_rst));
    if max(abs(error_tau)) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (invdyn_ne_T sim): FAILED!!! \n')
else
    fprintf('Test (invdyn_ne_T sim): PASS!!! \n')
end
fprintf('Time cost: \n')
fprintf('invdyn_ne_T:                  %f \n',t1/iter)
fprintf('invdyn_ne_T.t:                %f \n',t2/iter)
fprintf('------------------------ \n')

%% Test forward dynamic and mass matrix 
% 
test_failed = 0;
t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;
for iter = 1:100
    q = rand(hand.nj,1);

    hand.update_hand(q);
    hand.update_sim_par;
    hand.update_hand_par_dyn;
    hand_rst = hand.update_rst_model;
    w_T_all_frames = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);
    tic 
    [qDD,M_for,C_for,G_for] = hand.fordyn_ne_hand_w_end(q);
    t1 = t1+toc;
    tic
    M = invdyn_lag_T_M_Hand(w_T_all_frames, hand.sim.q_index,hand.sim.n_links,...
        hand.sim.l_index_in_frame,hand.sim.q_index_in_frame, hand.sim.Mass, hand.sim.CoM, hand.sim.I);
    t2 = t2+toc;
    tic
    M_rst = hand_rst.massMatrix(q);
    t3 = t3+toc;
    tic
    M_sim =  hand_function_test_invdyn_lag_T_M_Hand(w_T_all_frames);
    t4 = t4+toc;
    tic
    error_M1 = M_for - M;
    error_M2 = M_rst - M;
    error_M3 = M_rst - M_sim;
    error_M = [error_M1;error_M2;error_M3];
    if max(abs(error_M(:))) > 1e-8
        test_failed = 1;
        break
    end

end
if test_failed
    fprintf('Test (Mass matrix): FAILED!!! \n')
else
    fprintf('Test (Mass matrix): PASS!!! \n')
end
fprintf('Time cost: \n')
fprintf('fordyn_ne_hand_w_end:            %f \n',t1/iter)
fprintf('invdyn_lag_T_M_Hand:             %f \n',t2/iter)
fprintf('invdyn_lag_T_M_Hand.t:           %f \n',t4/iter)
fprintf('rst toolbox:                     %f \n',t3/iter)
fprintf('------------------------ \n')


%% Test forward dynamic and Gravity vector
test_failed = 0;
t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;
for iter = 1:100
    q = rand(hand.nj,1);

    hand.update_hand(q);
    hand.update_sim_par;
    hand.update_hand_par_dyn;
    hand_rst = hand.update_rst_model;
    w_T_all_frames = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);
    tic 
    [qDD,M_for,C_for,G_for] = hand.fordyn_ne_hand_w_end(q);
    t1 = t1+toc;
    tic
    G = invdyn_lag_T_G_Hand(w_T_all_frames, hand.sim.q_index,hand.sim.n_links,...
        hand.sim.l_index_in_frame,hand.sim.q_index_in_frame, hand.sim.Mass, hand.sim.CoM, hand.sim.g);
    t2 = t2+toc;
    tic
    G_rst = hand_rst.gravityTorque(q);
    t3 = t3+toc;
    tic
    G_sim =  hand_function_test_invdyn_lag_T_G_Hand(w_T_all_frames);
    t4 = t4+toc;
    
    error_G1 = G_for - G;
    error_G2 = G_rst - G;
    error_G3 = G_rst - G_sim;
    error_G = [error_G1;error_G2;error_G3];
    if max(abs(error_G(:))) > 1e-8
        test_failed = 1;
        break
    end

end
if test_failed
    fprintf('Test (Gravity vector): FAILED!!! \n')
else
    fprintf('Test (Gravity vector): PASS!!! \n')
end
fprintf('Time cost: \n')
fprintf('fordyn_ne_hand_w_end:            %f \n',t1/iter)
fprintf('invdyn_lag_T_G_Hand:             %f \n',t2/iter)
fprintf('invdyn_lag_T_G_Hand.t:           %f \n',t4/iter)
fprintf('rst toolbox:                     %f \n',t3/iter)
fprintf('------------------------ \n')

%% Test forward dynamic and coriolis vector
test_failed = 0;
t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    qd = rand(hand.nj,1);
    hand.update_hand(q);
    hand.update_sim_par;
    hand.update_hand_par_dyn;
    hand_rst = hand.update_rst_model;
    w_T_all_frames = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);
    tic 
    [qDD,M_for,C_for,G_for] = hand.fordyn_ne_hand_w_end(q,qd);
    t1 = t1+toc;
    tic
    C = invdyn_ne_T_C(w_T_all_frames,qd, hand.sim.n_links,hand.sim.q_index,...
        hand.sim.Mass, zeros(6,1),zeros(6,1),hand.sim.CoM,hand.sim.I);
    t2 = t2+toc;
    tic
    C_rst = hand_rst.velocityProduct(q,qd);
    t3 = t3+toc;
    tic
    C_sim =  hand_function_test_invdyn_ne_T_C(w_T_all_frames,qd,zeros(6,1),zeros(6,1));
    t4 = t4+toc;
    
    error_C1 = C_for - C;
    error_C2 = C_rst - C;
    error_C3 = C_rst - C_sim;
    error_C = [error_C1;error_C2;error_C3];
    if max(abs(error_C(:))) > 1e-8
        test_failed = 1;
        break
    end

end
if test_failed
    fprintf('Test (Coriolis vector): FAILED!!! \n')
else
    fprintf('Test (Coriolis vector): PASS!!! \n')
end
fprintf('Time cost: \n')
fprintf('fordyn_ne_hand_w_end:            %f \n',t1/iter)
fprintf('invdyn_ne_T_C:                   %f \n',t2/iter)
fprintf('invdyn_ne_T_C.t:                 %f \n',t4/iter)
fprintf('rst toolbox:                     %f \n',t3/iter)
fprintf('------------------------ \n')

