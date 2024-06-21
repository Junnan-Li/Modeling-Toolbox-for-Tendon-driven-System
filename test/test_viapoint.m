% test script for viapoint and muscle object functionalities within the hand


clear all
close all
clc

%% create hand 
hand = create_hand_random('hand_IK_test', [2,2,2,4] );
hand_rst = hand.update_rst_model;
q = rand(hand.nj,1);
hand.update_hand(q);

par_plot = hand.plot_parameter_init();
par_plot.axis_len = 0.4;
figure(4)
hand.plot_hand(par_plot)
hand_rst.show(q,'Frames','on');
axis equal


%% add VP 

hand.bas

finger_r.add_ViaPoint('origin_1', 1, [0;0;0.02])
finger_r.add_ViaPoint('ViaPoint_1', 2, [0;0;0.02])
finger_r.add_ViaPoint('ViaPoint_2', 3, [0;0;0.02])
finger_r.add_ViaPoint('Insertion', 4, [0.05;0;0])
finger_r.update_finger(rand(4,1));

figure
finger_r.plot_finger;
parstr = finger_index.plot_parameter_init;
parstr.markersize = 10;
parstr.markercolor = 'r';
finger_r.plot_viapoints(parstr);
axis equal
[w_p_viapoints_all,~] = finger_r.get_p_all_viapoints;

