%   example script for creating an example of finger

%% create finger with tendons

clear all 
close all
clc

% define a finger
finger_dimension_r = [0,0.05,0.03,0.02]; % in meter
finger_dimension_t = [0,0.03,0.02,0.012]; % in meter

finger_r = Finger('Index', 'type','R_RRRR', 'l_links',finger_dimension_r); 
finger_t = Finger('thumb', 'type','R_RRRR', 'l_links',finger_dimension_t);

mdh_default_struct = finger_r.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
finger_r.set_mdh_parameters(mdh_matrix);

mdh_matrix_t = mdh_matrix;
mdh_matrix_t(2:5,2) = finger_dimension_t';
finger_t.set_mdh_parameters(mdh_matrix_t);

% return
%% set states
% set base position and orientation
finger_r.w_p_base = 4*zeros(3,1);
finger_r.w_R_base = euler2R_XYZ([0,-pi/6,0]);

finger_t.w_p_base = [-0.02;0;-0.08];
finger_t.w_R_base = euler2R_XYZ([pi,0,0]);
% init joint configurations
q_0 = [0;0;0;0];

% udpate finger with given joint configurations
finger_r.update_finger(q_0);
finger_t.update_finger(q_0);
% load rst model from finger class
rst_model_r = finger_r.rst_model;
rst_model_t = finger_t.rst_model;

% plot 2 fingers
p_link_all_w_r = finger_r.get_p_all_links;
p_link_all_w_t = finger_t.get_p_all_links;
figure(1)

plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','r');
hold on
plot3(p_link_all_w_t(1,:)',p_link_all_w_t(2,:)',p_link_all_w_t(3,:)','o-','Color','b');
grid on
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

%% set dynamic parameters
% link index:
%   2: PP
%   3: MP
%   4: DP

finger_r.list_links(1).set_mass(0); % in kg
finger_r.list_links(1).set_inertia([0,0,0,0,0,0]); 

finger_r.list_links(2).set_mass(0.05); % in kg
finger_r.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(3).set_mass(0.03); % in kg
finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(4).set_mass(0.03); % in kg
finger_r.list_links(4).set_inertia([0.5,0.2,0.2,0,0,0]); 

% finger_r.list_links(3).set_mass(0.03); % in kg
% finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 
% update dynamic parameters
finger_r.update_finger_par_dyn;
% finger_r.update_finger(q_0);

finger_t.list_links(2).set_mass(0.05); % in kg
finger_t.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_t.list_links(3).set_mass(0.03); % in kg
finger_t.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_t.list_links(4).set_mass(0.03); % in kg
finger_t.list_links(4).set_inertia([0.5,0.2,0.2,0,0,0]); 

% update dynamic parameters
finger_t.update_finger_par_dyn;
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

if finger_t.nt == 0
    finger_t.add_tendon('Flex_1', [1,1,1,1]);
    finger_t.add_tendon('Flex_2', [1,1,1,0]);
    finger_t.add_tendon('Flex_3', [1,1,0,0]);
    finger_t.add_tendon('Flex_4', [1,0,0,0]);
    finger_t.add_tendon('Ext_1', [-1,-1,-1,-1]);
    finger_t.add_tendon('Ext_2', [-1,-1,-1,0]);
    finger_t.add_tendon('Ext_3', [-1,-1,0,0]);
    finger_t.add_tendon('Ext_4', [-1,0,0,0]);
end
% finger_r.set_tendon_par_MA_poly3(1,1,[0,0,0.01,0.03]);
% finger_r.set_tendon_par_MA_poly3(3,1,[0,0,0.01,0.3]);
% finger_r.set_tendon_par_MA_poly3(3,2,[0,0,0.01,0.035]);
% finger_r.set_tendon_par_MA_poly3(4,1,[0,0,0.001,0.02]);

finger_r.update_finger(q_0);
finger_r.update_M_coupling(q_0);
finger_r.M_coupling;

finger_t.update_finger(q_0);
finger_t.update_M_coupling(q_0);
finger_t.M_coupling;


% set joint limits
finger_r.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints

finger_r.update_joints_info;

finger_t.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_t.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_t.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints

finger_t.update_joints_info;

% add contacts
if finger_r.nc == 0
    for i = 1:finger_r.nl
        finger_r.list_links(i).add_contact([finger_r.list_links(i).Length/2 0 0]');
    end
end
finger_r.update_list_contacts; % update link

% add contacts
if finger_t.nc == 0
    for i = 1:finger_r.nl
        finger_t.list_links(i).add_contact([finger_t.list_links(i).Length/2 0 0]');
    end
end
finger_t.update_list_contacts; % update link

return





