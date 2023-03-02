% script for forward dynamic simulation and control and visualised in Unity
% 
% 
% 
% Junnan Li, junnan.li@tum.de, 02.2023



%% create finger with tendons

clear all 
close all
clc
% define a finger
finger_dimension = [0.05,0.03,0.02]; % in meter

% finger_r = Finger('Index', 'RRRR', finger_dimension); 

finger_dimension = [0,0.05,0.03,0.02]; % in meter

finger_r = Finger('Index', 'type','R_RRRR', 'l_links',finger_dimension); 

mdh_default_struct = finger_r.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
finger_r.set_mdh_parameters(mdh_matrix);
%% set states
% set base position and orientation
finger_r.w_p_base = 4*zeros(3,1);
finger_r.w_R_base = euler2R_XYZ(zeros(1,3));

% init joint configurations
q_0 = [0;0.1;0.1;0.1];

% udpate finger with given joint configurations
finger_r.update_finger(q_0);
% load rst model from finger class
rst_model_r = finger_r.rst_model;

%% set dynamic parameters
% link index:
%   1: PP
%   2: MP
%   3: DP

finger_r.list_links(1).set_mass(0.05); % in kg
finger_r.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(2).set_mass(0.03); % in kg
finger_r.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(3).set_mass(0.03); % in kg
finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

% update dynamic parameters
finger_r.update_finger_par_dyn;
% finger_r.update_finger(q_0);

%% add tendons
if finger_r.nt == 0
    finger_r.add_tendon('Flex_1', [1,1,1,1]);
    finger_r.add_tendon('Flex_2', [1,1,1,0]);
    finger_r.add_tendon('Flex_3', [1,1,0,0]);
    finger_r.add_tendon('Flex_4', [1,0,0,0]);
    finger_r.add_tendon('Ext_1', [-1,-1,-1,-1]);
    finger_r.add_tendon('Ext_2', [-1,-1,-1,0]);
    finger_r.add_tendon('Ext_3', [-1,-1,0,0]);
    finger_r.add_tendon('Ext_4', [-1,0,0,0]);
end

% finger_r.set_tendon_par_MA_poly3(1,1,[0,0,0.01,0.03]);
% finger_r.set_tendon_par_MA_poly3(3,1,[0,0,0.01,0.3]);
% finger_r.set_tendon_par_MA_poly3(3,2,[0,0,0.01,0.035]);
% finger_r.set_tendon_par_MA_poly3(4,1,[0,0,0.001,0.02]);

finger_r.update_finger(q_0);
finger_r.update_M_coupling(q_0);
finger_r.M_coupling;

% set joint limits
finger_r.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints

finger_r.update_joints_info;


% add contacts
if finger_r.nc == 0
    for i = 1:finger_r.nl
        finger_r.list_links(i).add_contact([finger_r.list_links(i).Length/2 0 0]');
    end
end
finger_r.update_list_contacts; % update link


%% simulation settings
n_q = finger_r.nj;
% finger object: finger_r
q_init = [0;0;0;0]*pi/180;
qD_init = [0;0;0;0]*pi/180;

finger_passive_stiffness = zeros(n_q,1);
finger_passive_damping = zeros(n_q,1);

gravity_compensation = 1;

% controller

Ks = [100;100;100;100];
Kd = [1;1;1;1];

F_ext = zeros(6,n_q+2);
mdh_ne = mdh_struct_to_matrix(finger_r.mdh_ori, 1);
mdh_ne(1:finger_r.nj,3) = mdh_ne(1:finger_r.nj,3); 


sim('./interface/Unity/simulink_unity.slx');