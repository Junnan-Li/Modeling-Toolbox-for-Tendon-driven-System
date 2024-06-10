% create an example hand


close all
clear all
clc

%% Create arm
% 2 dof base with a arbitary alpha in mdh parameters
% 
theta_rand = rand(1);
mdh_parameter1 = [0,0,0,0;theta_rand,0.5,0,0;-theta_rand,1.5,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter1, 1);
base_1 = Finger('base_1', 'mdh',mdh_struct );
base_1.set_base([0,0,0]',euler2R_XYZ([0,0,0]));

% define base dynamic parameters
base_1.list_links(1).set_mass(2);
base_1.list_links(1).set_com([0.2,0,0]');
base_1.list_links(2).set_mass(1);
base_1.list_links(2).set_com([0.3,0,0]');
base_1.update_finger_par_dyn;

%% finger 1
mdh_parameter2 = [0,0,0,0;pi/2,0.8,0,0;-pi/2,1,0,0;0,0.5,0,0;0,0.3,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter2, 1);
finger_1 = Finger('finger1', 'mdh',mdh_struct );
finger_1.set_base(rand(3,1),euler2R_XYZ(rand(3,1)));

q0_2 = rand(finger_1.nj,1);
finger_1.set_base_dynpar(1,[-0.2,0,0]',ones(6,1))
finger_1.list_links(1).set_mass(0.1);
finger_1.list_links(1).set_com([0.05,0,0]');
finger_1.list_links(2).set_mass(2);
finger_1.list_links(2).set_com([0.1,0,0]');
finger_1.list_links(3).set_mass(3);
finger_1.list_links(3).set_com([0.2,0,0]');
finger_1.list_links(4).set_mass(4);
finger_1.list_links(4).set_com([0.1,0,0]');
finger_1.update_finger_par_dyn;
finger_1.update_finger(q0_2);

%% finger 2
mdh_parameter3 = [0,0,0,0;0,0.2,0,0;pi/2,1,0,0;-pi/2,1,0,0;0,0.5,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter3, 1);
finger_2 = Finger('finger2', 'mdh',mdh_struct );
finger_2.set_base([2,0,0]',euler2R_XYZ([0,0,0]));

q0_f2 = rand(finger_2.nj,1);
finger_2.set_base_dynpar(1,[-0.2,0,0]',ones(6,1))
finger_2.list_links(1).set_mass(0.2);
finger_2.list_links(1).set_com([0.05,0,0]');
finger_2.list_links(2).set_mass(2);
finger_2.list_links(2).set_com([0.2,0,0]');
finger_2.list_links(3).set_mass(2);
finger_2.list_links(3).set_com([0.2,0,0]');
finger_2.list_links(4).set_mass(3);
finger_2.list_links(4).set_com([0.1,0,0]');
finger_2.update_finger_par_dyn;
finger_2.update_finger(q0_f2);

%% create hand with two fingers

hand = Hand('hand_example');
hand.add_base(base_1);
hand.add_finger(finger_1);
hand.add_finger(finger_2);
q_hand = rand(hand.nj,1);
hand.update_hand(q_hand);

