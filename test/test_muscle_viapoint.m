% test muscles and viapoint class

close all
clear all
clc

run examples\create_shadow_hand.m


%% add muscles

 % define muscles and associated viapoints
finger_index.add_Muscle('Muscle_1');
finger_index.add_Muscle('Muscle_2');
finger_index.add_ViaPoint_to_muscle('origin_1', 1, [0;0;0],1)
finger_index.add_ViaPoint_to_muscle('ViaPoint_1', 2, [0.01;0.01;0],1)
finger_index.add_ViaPoint_to_muscle('ViaPoint_1', 2, [0.03;0.01;0],1)
finger_index.add_ViaPoint_to_muscle('ViaPoint_1', 3, [0.01;0.01;0],1)
finger_index.add_ViaPoint_to_muscle('ViaPoint_1', 3, [0.02;0.01;0],1)
finger_index.add_ViaPoint_to_muscle('Insertion', 4, [0.01;0;0],1)

finger_index.add_ViaPoint_to_muscle('origin_1', 1, [0;0;0],2)
finger_index.add_ViaPoint_to_muscle('ViaPoint_1', 2, [0.015;-0.01;0],2)
finger_index.add_ViaPoint_to_muscle('ViaPoint_1', 2, [0.03;-0.01;0],2)
finger_index.add_ViaPoint_to_muscle('ViaPoint_1', 3, [0.01;-0.01;0],2)
finger_index.add_ViaPoint_to_muscle('ViaPoint_1', 3, [0.02;-0.01;0],2)
finger_index.add_ViaPoint_to_muscle('Insertion', 4, [0.02;0;0],2)
finger_index.update_finger(1.5*rand(4,1));
Muscle_length = finger_index.cal_all_Muscle_Length;



figure

finger_index.plot_finger;
parstr = finger_index.plot_parameter_init;
parstr.markersize = 10;
parstr.markercolor = 'r';
% finger_index.plot_viapoints(parstr);
finger_index.plot_muscles;
axis equal
grid on
[w_p_viapoints_all,~] = finger_index.get_p_all_viapoints;
[w_p_viapoints_all,~] = finger_index.get_p_muscle_viapoints([1,2]); 

% MA = finger_index.cal_all_Muscle_momentarm_1st_c(0.001) % calculate moment arm value with step 0.001
MA_sub = finger_index.cal_selected_Muscle_momentarm_1st_c(0.001,2)
MA_sub2 = finger_index.cal_selected_Muscle_momentarm_1st_c(0.01,2)
return
%%
VP_1 = ViaPoint('VP_1');

finger_index.add_ViaPoint('origin_1', 1, [0;0;0])
finger_index.add_ViaPoint('ViaPoint_1', 2, [0.01;0.01;0])
finger_index.add_ViaPoint('ViaPoint_1', 2, [0.03;0.01;0])
finger_index.add_ViaPoint('ViaPoint_1', 3, [0.01;0.01;0])
finger_index.add_ViaPoint('ViaPoint_1', 3, [0.02;0.01;0])
finger_index.add_ViaPoint('Insertion', 4, [0.01;0;0])
finger_index.update_finger(rand(4,1));

figure
finger_index.plot_finger;
parstr = finger_index.plot_parameter_init;
parstr.markersize = 10;
parstr.markercolor = 'r';
finger_index.plot_viapoints(parstr);
axis equal
[w_p_viapoints_all,~] = finger_index.get_p_all_viapoints;






