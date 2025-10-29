%   example script for creating an example of finger mdh

%% create finger with tendons

clear all 
close all
clc

%% define a finger with conventianal configuration
% four joints
finger_dimension = [1,1]; % in meter
finger_r = Finger('Index', 'type','R_RR', 'l_links',finger_dimension); 
% finger_r = Finger('hand_one','mdh',)
mdh_default_struct = finger_r.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
% mdh_matrix(2,1) = -pi/2;
% mdh_matrix(2,4) = 1;
% finger_r.set_mdh_parameters(mdh_matrix);

%%

% mdh_struct = mdh_matrix_to_struct(mdh_parameter, 1);
% finger_r = Finger('Index', 'mdh',mdh_struct );


%% basic settings
% set base position and orientation
finger_r.set_base_p(4*rand(3,1));
finger_r.set_base_R(euler2R_XYZ(zeros(1,3)));
% init joint configurations
q_0 = [1,1]';

% udpate finger with given joint configurations
finger_r.update_finger(q_0);
finger_r.update_rst_model;

% set dynamic parameters

finger_r.list_links(1).set_mass(0.04); % in kg
finger_r.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(2).set_mass(0.03); % in kg
finger_r.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

% update dynamic parameters
finger_r.update_finger_par_dyn;
finger_r.update_rst_model;
% finger_r.update_finger(q_0);