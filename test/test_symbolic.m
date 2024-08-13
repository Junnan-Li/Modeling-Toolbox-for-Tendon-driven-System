% test symbolic expression generation
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


%% 

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

%%
hand = create_hand_random('hand_Obstacle_Set', [2,2,4,4] );

q = rand(hand.nj,1);
hand.update_hand(q);
w_T_links_all_class = zeros(4,4,hand.nj);
for i = 1:hand.nl
    w_T_links_all_class(:,:,i) = hand.list_links(i).get_w_T_Link_inhand;
end

% w_T_all_links_sym = hand.sym_T_all_links_from_q(0)

hand.update_sim_mdh;
