% test non-class function used for simulink computation
% 
%           
% 


clear all
close all
clc


%% create random finger
finger = create_finger_random('finger_example', 5);

% generate symbolic 
% w_T_all_sym = finger.sym_T_all_links_from_q(1); 
% codegen T_all_links_from_q_finger_example.m -args {zeros(finger.nj,1)}


%% Finger class functions
q  = rand(finger.nj,1);
tic
finger.update_finger(q);
w_T_all = finger.get_T_all_links();
t1 = toc;
% tic
% w_T_all_sym = T_all_links_from_q_finger_example(q(:));
% t2 = toc;
% tic
% w_T_all_sym_opt = T_all_links_from_q_opt_finger_example(q(:));
% t3 = toc;
% tic
% w_T_all_sym_opt = T_all_links_from_q_finger_example_mex(q(:));
% t4 = toc;

fprintf('Finger function Time cost: \n')
fprintf('object function:            %f \n',t1)
% fprintf('symbolic function:          %f \n',t2)
% fprintf('optimized symbolc function: %f \n',t3)
% fprintf('compiled symbolic function: %f \n',t4)
fprintf('------------------------------------- \n')
%% generate hand 
hand = create_hand_random('hand_function_test', [2,2,4,4] );

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
    fprintf('Test (Links transformation): failed! \n')
else
    fprintf('Test (Links transformation): pass! \n')
end

fprintf('Hand function Time cost: \n')
fprintf('object function:            %f \n',t1/iter)
fprintf('sim function:               %f \n',t2/iter)
fprintf('------------------------ \n')
%% Transformation matrix for all frames
% sim_w_T_all_frames_from_q.m
test_failed = 0;
% t1 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    hand.update_hand(q);
    [w_T_all_frames,w_T_all_links] = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);
   
    T = hand.get_w_T_all();
    w_T_links_inhand = hand.get_w_T_links_inhand;

    T_f_error = w_T_all_frames-T;
    T_l_error = w_T_all_links - w_T_links_inhand;
    if max(abs([T_f_error(:);T_l_error(:)])) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (frames transformation): failed! \n')
else
    fprintf('Test (frames transformation): pass! \n')
end
fprintf('------------------------ \n')
%% Inverse dynamic 

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
        hand.sim.Mass, X_base, XD_base,XDD_base, F_ext, hand.sim.CoM, hand.sim.I, hand.sim.g);
    t1 = t1 + toc;
    tau_rst = hand_rst.inverseDynamics(q,qd,qdd);
    error_tau = max(abs(tau_obj - tau_rst));

    if max(abs(error_tau)) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (Inverse dynamic): failed! \n')
else
    fprintf('Test (Inverse dynamic): pass! \n')
end
fprintf('Hand ID Time cost: \n')
fprintf('invdyn_ne_T:            %f \n',t1/iter)
fprintf('------------------------ \n')
%%

hand.create_sim_functions_hand
test_failed = 0;
% t1 = 0;
for iter = 1:100
    q = rand(hand.nj,1);
    [w_T_all_frames,w_T_all_links] = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);
    [w_T_all_frames_t,w_T_all_links_t] = hand_function_test_sim_w_T_all_frames_from_q(q);
    
    T_f_error = w_T_all_frames-w_T_all_frames_t;
    T_l_error = w_T_all_links - w_T_all_links_t;
    if max(abs([T_f_error(:);T_l_error(:)])) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (sim transformation function): failed! \n')
else
    fprintf('Test (sim transformation function): pass! \n')
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
        hand.sim.Mass, X_base, XD_base,XDD_base, F_ext, hand.sim.CoM, hand.sim.I, hand.sim.g);
    t1 = t1 + toc;
    tic
    tau_sim = hand_function_test_invdyn_ne_T(w_T_all_frames,qd,qdd, X_base, XD_base,XDD_base, F_ext);
    t2 = t2 + toc;
    tau_rst = hand_rst.inverseDynamics(q,qd,qdd);
    error_tau = max(abs(tau_obj - tau_sim));
    error_tau2 = max(abs(tau_obj - tau_rst));
    if max(abs(error_tau)) > 1e-8
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test (Inverse dynamic): failed! \n')
else
    fprintf('Test (Inverse dynamic): pass! \n')
end
fprintf('Hand ID Time cost: \n')
fprintf('invdyn_ne_T    :            %f \n',t1/iter)
fprintf('invdyn_ne_T sim:            %f \n',t2/iter)
fprintf('------------------------ \n')

return
%% Test forward dynamic and individual terms
% 

q = rand(hand.nj,1);
qd = rand(hand.nj,1);
qdd = rand(hand.nj,1);
hand.update_hand(q);
hand.update_sim_par;
hand.update_hand_par_dyn;
hand_rst = hand.update_rst_model;
[w_T_all_frames,w_T_all_links] = sim_w_T_all_frames_from_q(hand.sim.mdh,hand.sim.mdh_index,...
        hand.sim.q_index,hand.sim.w_T_b,hand.sim.n_links,q);


[qDD,M,C,G] = hand.fordyn_ne_hand_w_end(q);
M_sym = invdyn_lag_T_M(w_T_all_links, hand.sim.Mass, hand.sim.CoM, hand.sim.I);
error_M = double(M_sym) - M;
max(error_M(:));

M_rst = hand_rst.massMatrix


