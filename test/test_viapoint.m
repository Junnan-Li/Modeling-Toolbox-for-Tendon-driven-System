% test viapoints of fingers and hand


clear all
close all
clc

%% test viapoint in the finger object
run examples\create_finger.m

VP_1 = ViaPoint('VP_1')

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

