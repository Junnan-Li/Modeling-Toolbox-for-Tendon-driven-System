% test script for viapoint and muscle object functionalities within the hand
% 
% 

clear all
close all
clc

%% create hand 
hand = create_hand_random('hand_viapoint_test', [2,2,2,4] );
hand_rst = hand.update_rst_model;
q = rand(hand.nj,1);
hand.update_hand(q);

par_plot = hand.plot_parameter_init();
par_plot.axis_len = 0.4;
% figure(4)
% hand.plot_hand(par_plot)
% hand_rst.show(q,'Frames','on');
% axis equal

%% add VP 

% vp01 = hand.base(1).add_ViaPoint('VP01', 0, -0.5*rand(1,3));
% vp02 = hand.base(1).add_ViaPoint('VP02', 0, -0.5*rand(1,3));
% vp03 = hand.base(1).add_ViaPoint('VP03', 0, -0.5*rand(1,3));

vp1 = hand.base(1).add_ViaPoint('VP1', 1, 0.5*rand(1,3));
vp2 = hand.base(1).add_ViaPoint('VP2', 2, 0.5*rand(1,3));

vp3 = hand.base(2).add_ViaPoint('VP3', 1, 0.5*rand(1,3));
vp4 = hand.base(2).add_ViaPoint('VP4', 1, 0.5*rand(1,3));

vp01 = hand.list_fingers(1).add_ViaPoint('VP01', 0, -0.5*zeros(1,3));
vp02 = hand.list_fingers(1).add_ViaPoint('VP02', 0, -0.5*rand(1,3));
vp03 = hand.list_fingers(1).add_ViaPoint('VP03', 0, -0.5*rand(1,3));

vp5 = hand.list_fingers(1).add_ViaPoint('VP5', 1, 0.5*rand(1,3));
vp6 = hand.list_fingers(1).add_ViaPoint('VP6', 2, 0.5*rand(1,3));
vp7 = hand.list_fingers(1).add_ViaPoint('VP7', 3, 0.5*rand(1,3));
vp8 = hand.list_fingers(1).add_ViaPoint('VP8', 4, 0.5*rand(1,3));

vp9 = hand.list_fingers(2).add_ViaPoint('VP9', 1, 0.5*rand(1,3));
vp10 = hand.list_fingers(2).add_ViaPoint('VP10', 2, 0.5*rand(1,3));
vp11 = hand.list_fingers(2).add_ViaPoint('VP11', 3, 0.5*rand(1,3));
vp12 = hand.list_fingers(2).add_ViaPoint('VP12', 4, 0.5*rand(1,3));



hand.update_list_viapoints;
q = rand(hand.nj,1);
hand.update_hand(q);
% figure(1)
% hand.plot_hand(par_plot);
% hand.plot_hand_viapoints(par_plot);
% hand_rst.show(q,'Frames','on');



%% add muscles
Muscle_1 = Muscles("muscle_1");
hand.add_Muscle(Muscle_1);
Muscle_1.add_viapoints(vp01);
Muscle_1.add_viapoints(vp03);
Muscle_1.add_viapoints(vp1);
Muscle_1.add_viapoints(vp3);
Muscle_1.add_viapoints(vp5);
Muscle_1.add_viapoints(vp6);
Muscle_1.add_viapoints(vp7);
Muscle_1.add_viapoints(vp8);

Muscle_2 = Muscles("muscle_2");
hand.add_Muscle(Muscle_2);
Muscle_2.add_viapoints(vp02);
Muscle_2.add_viapoints(vp2);
Muscle_2.add_viapoints(vp4);
Muscle_2.add_viapoints(vp9);
Muscle_2.add_viapoints(vp10);
Muscle_2.add_viapoints(vp11);
Muscle_2.add_viapoints(vp12);

figure(2)
for i = 1:3
q = 0.2*rand(hand.nj,1);
hand.update_hand(q);
hand.plot_hand(par_plot);
hand.plot_hand_viapoints(par_plot);
hand.plot_hand_muscles(par_plot);
T_base = hand.list_fingers(1).get_w_T_base_inhand;
plot3(T_base(1,4),T_base(2,4),T_base(3,4),'rx')
end

l_mus_1 = hand.get_muscle_length_all;
tic
MA = hand.get_Muscle_Momentarm_1st_c([1,2]);
t1 = toc;


