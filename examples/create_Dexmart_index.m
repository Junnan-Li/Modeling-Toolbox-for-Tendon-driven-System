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

Dexmart_index = Finger('Index_Dexmart', 'type','R_RRRR', 'l_links',finger_index_dimension); 
mdh_default_struct = Dexmart_index.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
% mdh_matrix(1,4) = -0.00291;
Dexmart_index.set_mdh_parameters(mdh_matrix);
% 
% T = T_dh([0.04075,0*pi/180,0.082,-95*pi/180]);
Dexmart_index.w_p_base = zeros(3,1);
Dexmart_index.w_R_base = euler2R_XYZ([0;0;0]);


q_0 = [0.1,0.3,0.3,0.1]';
Dexmart_index.update_finger(q_0);



% set dynamic parameters

Dexmart_index.list_links(1).set_mass(0.02); % in kg
Dexmart_index.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

Dexmart_index.list_links(2).set_mass(0.03); % in kg
Dexmart_index.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

Dexmart_index.list_links(3).set_mass(0.02); % in kg
Dexmart_index.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

Dexmart_index.list_links(4).set_mass(0.02); % in kg
Dexmart_index.list_links(4).set_inertia([0.5,0.2,0.2,0,0,0]); 
% update dynamic parameters
Dexmart_index.update_finger_par_dyn;
Dexmart_index.update_rst_model;


% plot finger and com
plot_setting = Dexmart_index.plot_parameter_init;
plot_setting.axis_len = 0.01;
figure(1)
Dexmart_index.plot_finger(plot_setting)
plot_setting.markersize = 10;
Dexmart_index.plot_com(plot_setting)
axis equal


return

%% add tendons

if Dexmart_index.nt == 0
    Dexmart_index.add_tendon('Flex_1', [zeros(3,Dexmart_index.nj);0.0082,0.0082,0,0]);
    Dexmart_index.add_tendon('Flex_2', [zeros(3,Dexmart_index.nj);-0.0078,0.0078,0,0]);
    Dexmart_index.add_tendon('Flex_3', [zeros(3,Dexmart_index.nj);0,0,0.0063,0]);
    Dexmart_index.add_tendon('Ext_1', [zeros(3,Dexmart_index.nj);0,-0.0053,-0.0053,-0.0053]);
    Dexmart_index.add_tendon('Ext_2', [zeros(3,Dexmart_index.nj);0,0,-0.0035,0.005]);
end

Dexmart_index.update_finger(q_0);
Dexmart_index.update_M_coupling(q_0);
Dexmart_index.M_coupling;
% set joint limits
Dexmart_index.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
Dexmart_index.list_joints(2).q_limits = [-5,85]*pi/180; % abduction joints
Dexmart_index.list_joints(3).q_limits = [0,85]*pi/180; % abduction joints
Dexmart_index.list_joints(4).q_limits = [0,45]*pi/180; % abduction joints

Dexmart_index.update_joints_info;

%% add contact
if Dexmart_index.nc == 0
    for i = 1:Dexmart_index.nl-1
        b_R_i = Dexmart_index.list_links(i).base_R;
        b_p_i1 = Dexmart_index.list_links(i+1).base_p;
        b_p_i = Dexmart_index.list_links(i).base_p;
        Dexmart_index.list_links(i).add_contact((b_R_i'*(b_p_i1-b_p_i))/2);
%         finger_r.list_links(i).add_contact([finger_r.list_links(i).Length 0 0]');
    end
    mdh_ori = mdh_struct_to_matrix(Dexmart_index.mdh_ori(end,:), 1);
    T_end = T_mdh_multi(mdh_ori(end,:));
    Dexmart_index.list_links(Dexmart_index.nl).add_contact(T_end(1:3,4)/2);
end
Dexmart_index.update_list_contacts; % update link


return