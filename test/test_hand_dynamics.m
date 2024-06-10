% test Hand and associated dynamics


close all
clear all
clc


run examples\create_hand.m
%%


hand_rst = hand.update_rst_model;

q = rand(hand.nj,1);
hand.update_hand(q);

plot_par = hand.plot_parameter_init;
plot_par.axis_len = 0.5;
figure(1)
hand.plot_hand(plot_par)
axis equal
hand_rst.show(q,'Frames','on');

%% Mass matrix

M_rst = hand_rst.massMatrix(q);
tau = -hand_rst.inverseDynamics(q); % gravity torque


hand.list_fingers(1).invdyn_ne_w_end



