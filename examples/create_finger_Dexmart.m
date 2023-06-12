% create finger models with the DEXMART hand parameters

% source:
%       [1] G. Palli et al., “The DEXMART hand: Mechatronic design and experimental 
%       evaluation of synergy-based control for human-like grasping,” Int. J. Rob. Res.,
%       vol. 33, no. 5, pp. 799–824, 2014
% 
% 
%   Junnan Li, junnan.li@tum.de, MIRMI, 06.2023

%% create finger with tendons

clear all 
close all
clc

%% define fingers with conventianal configuration
% Index finger
finger_index_dimension = [0.018,0.038,0.028,0.0285]; % in meter

finger_index = Finger('Index_Dexmart', 'type','R_RRRR', 'l_links',finger_index_dimension); 
mdh_default_struct = finger_index.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
% mdh_matrix(1,4) = -0.00291;
finger_index.set_mdh_parameters(mdh_matrix);

T = T_dh([0.04075,0*pi/180,0.082,-95*pi/180]);
finger_index.w_p_base = T(1:3,4);
finger_index.w_R_base = T(1:3,1:3);

finger_index.update_finger([0.1,0.3,0.3,0.1]')

finger_index.print_finger

% middle finger
finger_middle_dimension = [0.018,0.04,0.028,0.0285]; % in meter

finger_mid = Finger('Mid_Dexmart', 'type','R_RRRR', 'l_links',finger_middle_dimension); 
mdh_default_struct = finger_mid.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
% mdh_matrix(1,4) = -0.00291;
finger_mid.set_mdh_parameters(mdh_matrix);

T = T_dh([0.01434,0*pi/180,0.086,-86*pi/180]);
finger_mid.w_p_base = T(1:3,4);
finger_mid.w_R_base = T(1:3,1:3);
finger_mid.update_finger([0.1,0.3,0.3,0.1]')




% finger_index.update_finger([0.5,0.1,0.2,0.3]');
% finger_index.print_finger
% axis equal
% grid on

% ring finger
% finger_index_dimension = [0.018,0.038,0.028,0.0285]; % in meter
% 
% finger_index = Finger('Index_Dexmart', 'type','R_RRRR', 'l_links',finger_index_dimension); 
% mdh_default_struct = finger_index.mdh_ori;
% mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
% mdh_matrix(2,1) = -pi/2;
% % mdh_matrix(1,4) = -0.00291;
% finger_index.set_mdh_parameters(mdh_matrix);
% 
% % finger_index.update_finger([0.5,0.1,0.2,0.3]');
% % finger_index.print_finger
% % axis equal
% grid on

%%
figure(1)

finger_index.print_finger
hold on
finger_mid.print_finger
% return
%% basic settings

finger_index.update_rst_model;

% set dynamic parameters

finger_index.list_links(1).set_mass(0.02); % in kg
finger_index.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_index.list_links(2).set_mass(0.03); % in kg
finger_index.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_index.list_links(3).set_mass(0.02); % in kg
finger_index.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_index.list_links(4).set_mass(0.02); % in kg
finger_index.list_links(4).set_inertia([0.5,0.2,0.2,0,0,0]); 
% update dynamic parameters
finger_index.update_finger_par_dyn;
finger_index.update_rst_model;
% finger_r.update_finger(q_0);

%% add tendons

if finger_index.nt == 0
    finger_index.add_tendon('Flex_1', [0.0082,0.0082,0,0]);
    finger_index.add_tendon('Flex_2', [-0.0078,0.0078,0,0]);
    finger_index.add_tendon('Flex_3', [0,0,0.0063,0]);
    finger_index.add_tendon('Ext_1', [0,-0.0053,-0.0053,-0.0053]);
    finger_index.add_tendon('Ext_2', [0,0,-0.0035,0.005]);
end

finger_index.update_finger(q_0);
finger_index.update_M_coupling(q_0);
finger_index.M_coupling;
% set joint limits
finger_index.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_index.list_joints(2).q_limits = [-5,85]*pi/180; % abduction joints
finger_index.list_joints(3).q_limits = [0,85]*pi/180; % abduction joints
finger_index.list_joints(4).q_limits = [0,45]*pi/180; % abduction joints

finger_index.update_joints_info;

if finger_mid.nt == 0
    finger_mid.add_tendon('Flex_1', [0.0082,0.0082,0,0]);
    finger_mid.add_tendon('Flex_2', [-0.0078,0.0078,0,0]);
    finger_mid.add_tendon('Flex_3', [0,0,0.0063,0]);
    finger_mid.add_tendon('Ext_1', [0,-0.0053,-0.0053,-0.0053]);
    finger_mid.add_tendon('Ext_2', [0,0,-0.0035,0.005]);
end

finger_mid.update_finger(q_0);
finger_mid.update_M_coupling(q_0);
finger_mid.M_coupling;
% set joint limits
finger_mid.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_mid.list_joints(2).q_limits = [-5,85]*pi/180; % 
finger_mid.list_joints(3).q_limits = [0,85]*pi/180; % 
finger_mid.list_joints(4).q_limits = [0,45]*pi/180; %
finger_mid.update_joints_info;
return
%% add contact
if finger_index.nc == 0
    for i = 1:finger_index.nl-1
        b_R_i = finger_index.list_links(i).base_R;
        b_p_i1 = finger_index.list_links(i+1).base_p;
        b_p_i = finger_index.list_links(i).base_p;
        finger_index.list_links(i).add_contact((b_R_i'*(b_p_i1-b_p_i))/2);
%         finger_r.list_links(i).add_contact([finger_r.list_links(i).Length 0 0]');
    end
    mdh_ori = mdh_struct_to_matrix(finger_index.mdh_ori(end,:), 1);
    T_end = T_mdh_multi(mdh_ori(end,:));
    finger_index.list_links(finger_index.nl).add_contact(T_end(1:3,4)/2);
end
finger_index.update_list_contacts; % update link


return