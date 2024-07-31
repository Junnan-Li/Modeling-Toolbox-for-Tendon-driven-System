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
t1 = toc

tic
w_T_all_sym = T_all_links_from_q_finger_example(q(:));
t2 = toc
tic
w_T_all_sym_opt = T_all_links_from_q_opt_finger_example(q(:));
t3 = toc

tic
w_T_all_sym_opt = T_all_links_from_q_finger_example_mex(q(:));
t4 = toc




