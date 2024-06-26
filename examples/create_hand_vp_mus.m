% test script for viapoint and muscle object functionalities within the hand


clear all
close all
clc

%% create hand 
hand = create_hand_random('hand_with_VP_Mus', [2,2,2,4] );
hand_rst = hand.update_rst_model;
q = rand(hand.nj,1);
hand.update_hand(q);

%% add VP 

vp1 = hand.base(1).add_ViaPoint('VP1', 1, 0.5*rand(1,3));
vp2 = hand.base(1).add_ViaPoint('VP2', 2, 0.5*rand(1,3));

vp3 = hand.base(2).add_ViaPoint('VP3', 1, 0.5*rand(1,3));
vp4 = hand.base(2).add_ViaPoint('VP4', 1, 0.5*rand(1,3));

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

%% add muscles

Muscle_1 = hand.add_Muscle("muscle_1");
Muscle_1.add_viapoints(vp1);
Muscle_1.add_viapoints(vp3);
Muscle_1.add_viapoints(vp5);
Muscle_1.add_viapoints(vp6);
Muscle_1.add_viapoints(vp7);
Muscle_1.add_viapoints(vp8);

Muscle_2 = hand.add_Muscle("muscle_2");
Muscle_2.add_viapoints(vp2);
Muscle_2.add_viapoints(vp4);
Muscle_2.add_viapoints(vp9);
Muscle_2.add_viapoints(vp10);
Muscle_2.add_viapoints(vp11);
Muscle_2.add_viapoints(vp12);