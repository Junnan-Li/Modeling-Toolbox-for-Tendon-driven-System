% test non-class function used for simulink computation
% 
%           
% 


clear all
close all
clc


%% create random finger
finger = create_finger_random('finger_example', 5);
w_T_all_sym = finger.sym_T_all_links_from_q(1);
% codegen T_all_links_from_q_finger_example.m -args {zeros(finger.nj,1)}


%% Finger class functions
q  = rand(finger.nj,1);
tic
finger.update_finger(q);
w_T_all = finger.get_T_all_links();
t1 = toc;
tic
w_T_all_sym = T_all_links_from_q_finger_example(q(:));
t2 = toc;
tic
w_T_all_sym_opt = T_all_links_from_q_opt_finger_example(q(:));
t3 = toc;
tic
w_T_all_sym_opt = T_all_links_from_q_finger_example_mex(q(:));
t4 = toc;

%% generate hand 
hand = create_hand_random('hand_Obstacle_Set', [2,2,4,4] );



% add 

%% transformation matrix of each links
q = rand(hand.nj,1);
hand.update_hand(q);
w_T_links_all_class = zeros(4,4,hand.nj);
for i = 1:hand.nl
    w_T_links_all_class(:,:,i) = hand.list_links(i).get_w_T_Link_inhand;
end

w_T_all_links_cla = hand.get_w_T_links_inhand;

hand.update_sim_mdh;

w_T_all_links = sim_w_T_all_links_from_q(hand.sim_mdh,hand.sim_mdh_index,...
                        hand.sim_q_index,hand.sim_w_T_b,hand.sim_n_links,q);
T_error1 = w_T_links_all_class-w_T_all_links_cla;
T_error2 = w_T_links_all_class-w_T_all_links;
T_error = [T_error1;T_error2];

if max(abs(T_error(:))) > 1e-8 
    fprintf('Test (Links transformation): failed! \n')
else
    fprintf('Test (Links transformation): pass! \n')
end
%%




