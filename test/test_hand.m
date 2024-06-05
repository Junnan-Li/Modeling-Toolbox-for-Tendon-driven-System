% test Hand and associated dynamics


close all
clear all
clc

%% Test generate fingers and wrist seperately and calculate J and M
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

% visualization
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
mdh_parameter2 = [0,0,0,0;pi/2,0.8,0,0;-pi/2,1,0,0;0,0.7,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter2, 1);
finger_1 = Finger('finger1', 'mdh',mdh_struct );
finger_1.set_base(rand(3,1),euler2R_XYZ(rand(3,1)));

q0_2 = rand(finger_1.nj,1);
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

% calculate the gravity torque using both methods 
gravTorq = gravityTorque(rst_model_finger_1,q0_2);
tau = finger_1.invdyn_ne_w_end( q0_2, 0*q0_2,0*q0_2,zeros(6,1));
G_error = abs(gravTorq-tau);
if max(abs(G_error(:))) > 1e-10 
    fprintf('Test (gravity torque vector): failed! \n')
else
    fprintf('Test (gravity torque vector): pass! \n')
end

% Jacobian calculation
jacobian = geometricJacobian(rst_model_finger_1,q0_2,rst_model_finger_1.BodyNames{end});
jacobian_mod = [jacobian(4:6,:);jacobian(1:3,:)]; % using RST function
J = finger_1.Jacobian_geom_b_end(q0_2); 
w_R_b = finger_1.w_R_base;
J_w = blkdiag(w_R_b,w_R_b)*J; % using class function
J_error = abs(jacobian_mod-J_w);
if max(abs(J_error(:))) > 1e-10 
    fprintf('Test (Jacobian): failed! \n')
else
    fprintf('Test (Jacobian): pass! \n')
end

% Mass matrix
[~,M_fd,~,G_fd] = finger_1.fordyn_ne_w_end(q0_2,0*q0_2, 0*q0_2, zeros(6,finger_1.nj+2),0);
M = massMatrix(rst_model_finger_1,q0_2); % using RST function
M_error = abs(M-M_fd);
if max(abs(M_error(:))) > 1e-10 
    fprintf('Test (Mass Matrix): failed! \n')
else
    fprintf('Test (Mass Matrix): pass! \n')
end

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
%% merge method 1 (ignore the finger base position and orientation)
% merge the mdh parameters 
% remove the first row of the finger mdh parameter

% close all
mdh_parameter_merge = [mdh_parameter1;mdh_parameter2(2:end,:)];
mdh_struct = mdh_matrix_to_struct(mdh_parameter_merge, 1);
merge_1 = Finger('merged', 'mdh',mdh_struct );
merge_1.set_base([0,0,0]',euler2R_XYZ([0,0,0]));
q0_merge = zeros(merge_1.nj,1);
q0_merge = [0,0,1.2,0,0]';
merge_1.update_finger(q0_merge);

merge_1.update_finger(q0_merge);
merge_1.plot_finger(parameters);
merge_1.plot_com;
axis equal
merge_1.update_rst_model
rst_modelmerge = merge_1.rst_model;
rst_modelmerge.show(q0_merge, "Frames","on")
rst_modelmerge.DataFormat = 'column';
% rst_modelmerge.Gravity = [0 0 -9.81];

% calculate the gravity torque using both methods 
gravTorq = gravityTorque(rst_modelmerge,q0_merge);
tau = merge_1.invdyn_ne_w_end(q0_merge, 0*q0_merge,0*q0_merge,zeros(6,1));
G_error = abs(gravTorq-tau);
if max(abs(G_error(:))) > 1e-10 
    fprintf('Test merge (gravity torque vector): failed! \n')
else
    fprintf('Test merge (gravity torque vector): pass! \n')
end

% Jacobian calculation
jacobian = geometricJacobian(rst_modelmerge,q0_merge,rst_modelmerge.BodyNames{end});
jacobian_mod = [jacobian(4:6,:);jacobian(1:3,:)]; % using RST function
J = merge_1.Jacobian_geom_b_end(q0_merge); 
w_R_b = merge_1.w_R_base;
J_w = blkdiag(w_R_b,w_R_b)*J; % using class function
J_error = abs(jacobian_mod-J_w);
if max(abs(J_error(:))) > 1e-10 
    fprintf('Test (Jacobian): failed! \n')
else
    fprintf('Test (Jacobian): pass! \n')
end

% Mass matrix
[~,M_fd,~,G_fd] = merge_1.fordyn_ne_w_end(q0_merge,0*q0_merge, 0*q0_merge, zeros(6,merge_1.nj+2),0);
M = massMatrix(rst_modelmerge,q0_merge); % using RST function
M_error = abs(M-M_fd);
if max(abs(M_error(:))) > 1e-10 
    fprintf('Test (Mass Matrix): failed! \n')
else
    fprintf('Test (Mass Matrix): pass! \n')
end

%% merge method 2 (consider the finger base position and orientation)
% consider the base pos&orientation of the finger and fix the world origin to
% the wrist endeffector
% rst_model_base
% rst_model_finger_1, rst_model_finger_2
% 
close all
q_base = [20,20]'*pi/180;
q_finger1 = [20,30,0]'*pi/180;
q_finger2 = [20,30,0]'*pi/180;
q_merge = [q_base;q_finger1;q_finger2];

finger_1.set_base([1,0,0]',euler2R_XYZ([pi/4,0,0]));
finger_1.update_rst_model;
rst_model_finger_1 = finger_1.rst_model;
finger_1.update_finger(q_finger1)
finger_1.plot_finger()
axis equal
rst_model_finger_1.show(q_finger1)
hold on
finger_2.set_base([1,1,0]',euler2R_XYZ([-pi/4,0,0]));
finger_2.update_rst_model;
rst_model_finger_2 = finger_2.rst_model;
finger_2.update_finger(q_finger2)
finger_2.plot_finger()
hold on
rst_model_finger_2.show(q_finger2)


% reset the base pos
base_1.set_base([0,0,2]',euler2R_XYZ([0,0,0]))
base_1.update_rst_model
rst_model_base = base_1.rst_model;
rst_model_base.show(q_base)
rst_merge_2 = rst_model_base.copy; % copy base rst model as merged one
base_name = rst_merge_2.BodyNames{end};% the name of the endeffector that 
% will substitute the world origin of fingers
rst_merge_2.addSubtree(base_name,rst_model_finger_1)
rst_merge_2.addSubtree(base_name,rst_model_finger_2)
hold on

rst_merge_2.show(q_merge)


% calculate the gravity torque using both methods 
gravity_tau = gravityTorque(rst_merge_2,q_merge);


% Jacobian calculation
jacobian_f1 = geometricJacobian(rst_merge_2,q_merge,rst_model_finger_1.BodyNames{end});
jacobian_f1_mod = [jacobian_f1(4:6,:);jacobian_f1(1:3,:)]; 
jacobian_f2 = geometricJacobian(rst_merge_2,q_merge,rst_model_finger_2.BodyNames{end});
jacobian_f2_mod = [jacobian_f2(4:6,:);jacobian_f2(1:3,:)]; 

return
%% create hand with two fingers

hand = Hand('hand_exa');
hand.add_base(base_1);
hand.add_finger(finger_1);
% hand.add_finger(finger_2);
q_hand = rand(hand.nj,1);
hand.update_hand(q_hand);

hand_rst = hand.update_rst_model;
hand_rst.show(q_hand)
hold on
hand.plot_hand()
hand.update_hand(q_hand);
hand.plot_hand()
hand.plot_hand()
return
%% create hand 

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


