%   example script for creating an example of finger


% Junnan Li, junnan.li@tum.de, MIRMI, 02.2023
%% create finger with tendons

clear all 
close all
clc

% define a finger
finger_dimension_r = [0,0.05,0.03,0.02]; % in meter
finger_dimension_t = [0,0.03,0.02,0.012]; % in meter

finger_index = Finger('Index', 'type','R_RRRR', 'l_links',finger_dimension_r);   
finger_thumb = Finger('thumb', 'type','R_RRRR', 'l_links',finger_dimension_t);

mdh_default_struct = finger_index.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
finger_index.set_mdh_parameters(mdh_matrix);

mdh_matrix_t = mdh_matrix;
mdh_matrix_t(2:5,2) = finger_dimension_t';
finger_thumb.set_mdh_parameters(mdh_matrix_t);

% return
%% set states
% set base position and orientation
finger_index.w_p_base = 4*zeros(3,1);
finger_index.w_R_base = euler2R_XYZ([0,-pi/6,0]);
% finger_index.w_R_base = euler2R_XYZ([0,0,0]);


finger_thumb.w_p_base = [-0.01;0;-0.08];
finger_thumb.w_R_base = euler2R_XYZ([pi,0,0]);
% init joint configurations
q_0 = [0;0;0;0];

% udpate finger with given joint configurations
finger_index.update_finger(q_0);
finger_thumb.update_finger(q_0);
% load rst model from finger class
rst_model_r = finger_index.rst_model;
rst_model_t = finger_thumb.rst_model;

% plot 2 fingers
p_link_all_w_r = finger_index.get_p_all_links;
p_link_all_w_t = finger_thumb.get_p_all_links;
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
%  cylinder:
%   I_x = m*r^2/2;  I_y = I_z = 1/12*m*(3*r^2 + h^2)
%   2: PP l:5cm r=1cm M = 0.05kg 
%   3: MP
%   4: DP

finger_index.list_links(1).set_mass(0); % in kg
finger_index.list_links(1).set_inertia([0,0,0,0,0,0]); 

finger_index.list_links(2).set_mass(0.05); % in kg
finger_index.list_links(2).set_inertia([2.5e-6,1.16e-5,1.16e-5,0,0,0]); 

finger_index.list_links(3).set_mass(0.03); % in kg
finger_index.list_links(3).set_inertia([1.5e-6,3e-6,3e-6,0,0,0]); 

finger_index.list_links(4).set_mass(0.02); % in kg
finger_index.list_links(4).set_inertia([1e-6,1.17e-6,1.17e-6,0,0,0]); 

% finger_r.list_links(3).set_mass(0.03); % in kg
% finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 
% update dynamic parameters
finger_index.update_finger_par_dyn;
% finger_r.update_finger(q_0);

finger_thumb.list_links(2).set_mass(0.05); % in kg
finger_thumb.list_links(2).set_inertia([0.05,0.02,0.02,0,0,0]); 

finger_thumb.list_links(3).set_mass(0.03); % in kg
finger_thumb.list_links(3).set_inertia([0.05,0.02,0.02,0,0,0]); 

finger_thumb.list_links(4).set_mass(0.02); % in kg
finger_thumb.list_links(4).set_inertia([0.05,0.02,0.02,0,0,0]); 

% update dynamic parameters
finger_thumb.update_finger_par_dyn;
% finger_r.update_finger(q_0);

%% add tendons
finger_index_5_tendon = copy(finger_index);
if finger_index.nt == 0
    finger_index.add_tendon('Flex_1', [1,1,1,1]);
    finger_index.add_tendon('Flex_2', [-1,1,1,0]);
    finger_index.add_tendon('Flex_3', [1,1,0,0]);
    finger_index.add_tendon('Flex_4', [-1,0,0,0]);
    finger_index.add_tendon('Ext_1', [1,-1,-1,-1]);
    finger_index.add_tendon('Ext_2', [-1,-1,-1,0]);
    finger_index.add_tendon('Ext_3', [1,-1,0,0]);
    finger_index.add_tendon('Ext_4', [-1,0,0,0]);
end

if finger_index_5_tendon.nt == 0
    finger_index_5_tendon.add_tendon('Flex_1', [1,1,1,1]);
    finger_index_5_tendon.add_tendon('Flex_2', [1,1,1,0]);
    finger_index_5_tendon.add_tendon('Flex_3', [1,1,0,0]);
    finger_index_5_tendon.add_tendon('Flex_4', [1,0,0,0]);
    finger_index_5_tendon.add_tendon('Ext_1', [-1,-1,-1,-1]);
%     finger_index_5_tendon.add_tendon('Ext_2', [-1,-1,-1,0]);
%     finger_index_5_tendon.add_tendon('Ext_2', [-1,-1,0,0]);
    finger_index_5_tendon.add_tendon('Ext_2', [-1,0,0,0]);
end

if finger_thumb.nt == 0
    finger_thumb.add_tendon('Flex_1', [0.01,0.01,0.01,0.01]);
    finger_thumb.add_tendon('Flex_2', [0.01,0.01,0.01,0]);
    finger_thumb.add_tendon('Flex_3', [0.01,0.01,0,0]);
    finger_thumb.add_tendon('Flex_4', [0.01,0,0,0]);
    finger_thumb.add_tendon('Ext_1', [-0.01,-0.01,-0.01,-0.01]);
    finger_thumb.add_tendon('Ext_2', [-0.01,-0.01,-0.01,0]);
    finger_thumb.add_tendon('Ext_3', [-0.01,-0.01,0,0]);
    finger_thumb.add_tendon('Ext_4', [-0.01,0,0,0]);
end
% finger_r.set_tendon_par_MA_poly3(1,1,[0,0,0.01,0.03]);
% finger_r.set_tendon_par_MA_poly3(3,1,[0,0,0.01,0.3]);
% finger_r.set_tendon_par_MA_poly3(3,2,[0,0,0.01,0.035]);
% finger_r.set_tendon_par_MA_poly3(4,1,[0,0,0.001,0.02]);

finger_index.update_finger(q_0);
finger_index.update_M_coupling(q_0);
finger_index.M_coupling;

finger_index_5_tendon.update_finger(q_0);
finger_index_5_tendon.update_M_coupling(q_0);
finger_index_5_tendon.M_coupling;

finger_thumb.update_finger(q_0);
finger_thumb.update_M_coupling(q_0);
finger_thumb.M_coupling;


% set joint limits
finger_index.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_index.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_index.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints
finger_index.update_joints_info;

finger_index_5_tendon.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_index_5_tendon.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_index_5_tendon.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints
finger_index_5_tendon.update_joints_info;

finger_thumb.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_thumb.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_thumb.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints

finger_thumb.update_joints_info;

% add contacts
if finger_index.nc == 0
    for i = 1:finger_index.nl
        finger_index.list_links(i).add_contact([finger_index.list_links(i).Length/2 0 0]');
    end
end
finger_index.update_list_contacts; % update link

% add contacts
if finger_index_5_tendon.nc == 0
    for i = 1:finger_index_5_tendon.nl
        finger_index_5_tendon.list_links(i).add_contact([finger_index_5_tendon.list_links(i).Length/2 0 0]');
    end
end
finger_index_5_tendon.update_list_contacts; % update link

% add contacts
if finger_thumb.nc == 0
    for i = 1:finger_index.nl
        finger_thumb.list_links(i).add_contact([finger_thumb.list_links(i).Length/2 0 0]');
    end
end
finger_thumb.update_list_contacts; % update link

return





