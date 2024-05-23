% test Hand and associated dynamics


close all
clear all
clc

%% base 

theta_rand = rand(1);
mdh_parameter1 = [0,0,0,0;theta_rand,0.5,0,0;-theta_rand,1,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter1, 1);
base_1 = Finger('base_1', 'mdh',mdh_struct );
base_1.set_base([0,0,0]',euler2R_XYZ([0,0,0]));

base_1.list_links(1).set_mass(2);
base_1.list_links(1).set_com([0.2,0,0]');
base_1.list_links(2).set_mass(1);
base_1.list_links(2).set_com([0.3,0,0]');
base_1.update_finger_par_dyn;

q0_1 = rand(2,1);
base_1.update_finger(q0_1);
parameters = base_1.plot_parameter_init;
parameters.axis_len = 0.5;
base_1.plot_finger(parameters);
base_1.plot_com;
axis equal
base_1.update_rst_model
rst_model_base = base_1.rst_model;
rst_model_base.show(q0_1,"Frames","on")

%% finger 1
mdh_parameter2 = [0,0,0,0;0,0.2,0,0;pi/2,1,0,0;-pi/2,1,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter2, 1);
finger_1 = Finger('finger1', 'mdh',mdh_struct );
finger_1.set_base([2,0,0]',euler2R_XYZ([0,0,0]));

q0_2 = rand(3,1);
finger_1.list_links(1).set_mass(0.1);
finger_1.list_links(1).set_com([0.1,0,0]');
finger_1.list_links(2).set_mass(1);
finger_1.list_links(2).set_com([0.5,0,0]');
finger_1.list_links(3).set_mass(1);
finger_1.list_links(3).set_com([0.5,0,0]');
finger_1.update_finger_par_dyn;
finger_1.update_finger(q0_2);
finger_1.plot_finger(parameters);
finger_1.plot_com;
axis equal
finger_1.update_rst_model
rst_model_finger_1 = finger_1.rst_model;
rst_model_finger_1.show(q0_2,"Frames","on")

gravTorq = gravityTorque(rst_model_finger_1,q0_2)
tau = finger_1.invdyn_ne_w_end( q0_2, 0*q0_2,0*q0_2,zeros(6,1))

jacobian = geometricJacobian(rst_model_finger_1,q0_2,'endeffector')
J = finger_1.Jacobian_geom_b_end(q0_2)

%% finger 2
mdh_parameter3 = [0,0,0,0;0,0.2,0,0;pi/2,1,0,0;-pi/2,1,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter3, 1);
finger_2 = Finger('finger2', 'mdh',mdh_struct );
finger_2.set_base([2,0,0]',euler2R_XYZ([0,0,0]));

q0_f2 = rand(3,1);
finger_2.list_links(1).set_mass(0.2);
finger_2.list_links(1).set_com([0.2,0,0]');
finger_2.list_links(2).set_mass(2);
finger_2.list_links(2).set_com([0.5,0,0]');
finger_2.list_links(3).set_mass(2);
finger_2.list_links(3).set_com([0.5,0,0]');
finger_2.update_finger_par_dyn;
finger_2.update_finger(q0_f2);
finger_2.plot_finger(parameters);
finger_2.plot_com;
axis equal
finger_2.update_rst_model
rst_model_finger_2 = finger_2.rst_model;
rst_model_finger_2.show(q0_f2,"Frames","on")

[~,M_fd,~,G_fd] = finger_2.fordyn_ne_w_end(q0_f2,0*q0_f2, 0*q0_f2, zeros(6,finger_2.nj+2),0)
H = massMatrix(rst_model_finger_2,q0_f2)
gravTorq = gravityTorque(rst_model_finger_2,q0_f2)

jacobian = geometricJacobian(rst_model_finger_1,q0_f2,'endeffector')
J = finger_2.Jacobian_geom_b_end(q0_f2)
%% merge

mdh_parameter_merge = [mdh_parameter1;mdh_parameter2(2:end,:)];
mdh_struct = mdh_matrix_to_struct(mdh_parameter_merge, 1);
merge_1 = Finger('merge_1', 'mdh',mdh_struct );
merge_1.set_base([0,0,0]',euler2R_XYZ([0,0,0]));
q0_merge = 1.5*rand(merge_1.nj,1);
merge_1.update_finger(q0_merge);

merge_1.update_finger(q0_merge);
merge_1.plot_finger(parameters);
merge_1.plot_com;
axis equal
merge_1.update_rst_model
rst_modelmerge = merge_1.rst_model;
rst_modelmerge.show(q0_merge, "Frames","on")
rst_modelmerge.DataFormat = 'column';
rst_modelmerge.Gravity = [0 0 -9.81];



[~,M_fd,~,G_fd] = merge_1.fordyn_ne_w_end(q0_merge,0*q0_merge, 0*q0_merge, zeros(6,merge_1.nj+2),0)
H = massMatrix(rst_modelmerge,q0_merge)
gravTorq = gravityTorque(rst_modelmerge,q0_merge)
% tau = merge_1.invdyn_ne_w_end( q0_merge, 0*q0_merge, 0*q0_merge,zeros(6,1))

% jacobian = geometricJacobian(rst_modelmerge,q0_merge,'finger33')

%% hybrid structure
% rst_model_base
% rst_model_finger_1, rst_model_finger_2
% 

subtree = rst_model_finger_1.subtree('Link1')
subtree.Bodies{1}.Name = 'Subtree1';
body1 = rst_model_finger_1.Bodies{1}
addBody(rst_model_base,body1,'endeffector')

rst_model_base.addSubtree('endeffector',subtree,'ReplaceBase',true)

return
%%

run examples\create_shadow_hand.m

%% init hand 

hand = Hand('hand1');

finger2 = copy(finger_index)
finger2.name = 'figner2';
finger3 = copy(finger_index)
finger3.name = 'figner3';


finger_index.update_finger([0.3,1,0.1,0.1]');
[p_link_all_w1,~] = finger_index.get_p_all_links()
% [p_link_all_w2,~] = finger2.get_p_all_links()
finger2.update_finger(4*[0.1,0.1,0.1,0.1]');
[p_link_all_w1,~] = finger_index.get_p_all_links()
% [p_link_all_w2,~] = finger2.get_p_all_links()

finger3.update_finger(8*[0.1,0.1,0.1,0.1]');
[p_link_all_w1,~] = finger_index.get_p_all_links()

[p_link_all_w1,~] = finger_index.get_p_all_links();
[p_link_all_w2,~] = finger2.get_p_all_links();
[p_link_all_w3,~] = finger3.get_p_all_links();

return

hand.add_finger(finger_index);
hand.add_finger(finger2);
% hand.add_finger(finger3);

% hand.set_figner_base([0;0.01;0],eye(3),1);
% hand.set_figner_base([0;0;0],eye(3),2)
% hand.set_figner_base([0;-0.01;0],eye(3),3)

hand.update_hand(rand(hand.nj,1));

% hand.update_hand([0.1,0.1,0.1,0.1,0.2,0.2,0.2,0.2,0.3,0.3,0.3,0.3]');


a = hand.list_fingers(1);
b = hand.list_fingers(2);


hand.plot_hand()


