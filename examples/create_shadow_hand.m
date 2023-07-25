%   shadow hand model

% source:
%  https://github.com/shadow-robot/sr_common/tree/noetic-devel/sr_description/hand/xacro

% Junnan Li, junnan.li@tum.de, MIRMI, 02.2023
%% create finger with tendons

clear all 
% close all
clc

% define a finger
finger_dimension_r = [0,0.045,0.025,0.032]; % in meter
finger_dimension_t = [0,0.038,0,0.032,0.035]; % in meter
% finger_dimension_l = [0,0.038,0,0.032,0.035]; % in meter

finger_index = Finger('Index', 'type','R_RRRR', 'l_links',finger_dimension_r);   
finger_thumb = Finger('thumb', 'type','R_RRRRR', 'l_links',finger_dimension_t);
finger_little = Finger('thumb', 'type','R_RRRRR', 'l_links',finger_dimension_t);

mdh_default_struct = finger_index.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
finger_index.set_mdh_parameters(mdh_matrix);

mdh_default_struct = finger_thumb.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix_t = mdh_matrix;
mdh_matrix_t(2,1) = -pi/2;
mdh_matrix_t(2,3) = -pi/2;
mdh_matrix_t(4,1) = -pi/2;
mdh_matrix_t(2:end,2) = finger_dimension_t';
finger_thumb.set_mdh_parameters(mdh_matrix_t);

mdh_default_struct = finger_little.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix_l = mdh_matrix;
mdh_matrix_l(1,4) = 65.89*sin(55/180*pi)*1e-3;
mdh_matrix_l(2,1) = -pi/2;
mdh_matrix_l(2,2) = 65.89*cos(55/180*pi)*1e-3;
mdh_matrix_l(2,3) = -55/180*pi;
mdh_matrix_l(3,1) = pi/2;
mdh_matrix_l(3:end,2) = finger_dimension_r';
finger_little.set_mdh_parameters(mdh_matrix_l);

% return
%% set states
% set base position and orientation
R = euler2R_XYZ([0,-pi/6,0]);
finger_index.w_p_base = R*[0.129;0.034;0]; 
finger_index.w_R_base = R*euler2R_XYZ([0,0,0]);

finger_thumb.w_p_base = R*[0.063;0.034;0];
finger_thumb.w_R_base = R*euler2R_XYZ([0,pi/2,0])*euler2R_XYZ([-pi/4,0,0]);

finger_little.w_p_base = R*[0.05471;-0.032;0];
finger_little.w_R_base = R*euler2R_XYZ([-pi/2,55/180*pi,0]);
% init joint configurations
q_I_0 = [0;0;0;0]*pi/180;
q_T_0 = [0;0;0;0;0]*pi/180;
q_L_0 = [0;0;0;0;0]*pi/180;
% udpate finger with given joint configurations
finger_index.update_finger(q_I_0);
finger_thumb.update_finger(q_T_0);
finger_little.update_finger(q_L_0);


%% set dynamic parameters
% source:
%   https://github.com/shadow-robot/sr_common/blob/noetic-devel/sr_description/doc/HandInertia.md
% link index:
%  cylinder, 5mm radius, PLA 1.3g/cm^3 ~ 10g/1cm
% inertia:
%   cylinder: r = 0.005m 
%       x: 1/2*m*r^2
%       x: 1/4*m*r^2 + 1/12*m*l^2
%   

finger_index.list_links(1).set_mass(0); % in kg
finger_index.list_links(1).set_inertia([0,0,0,0,0,0]); 

finger_index.list_links(2).set_mass(0.03); % in kg
finger_index.list_links(2).set_inertia([1.8100e-06,1.0000e-05,9.8100e-06,0,0,0]); 

finger_index.list_links(3).set_mass(0.017); % in kg
finger_index.list_links(3).set_inertia([8.6842e-07,2.7257e-06,2.6761e-06,0,0,0]); 

finger_index.list_links(4).set_mass(0.012); % in kg
finger_index.list_links(4).set_inertia([5.3425e-07,1.0530e-06,9.3925e-07,0,0,0]); 

% finger_r.list_links(3).set_mass(0.03); % in kg
% finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 
% update dynamic parameters
finger_index.update_finger_par_dyn;


% thumb 
finger_thumb.list_links(1).set_mass(0); % in kg
finger_thumb.list_links(1).set_inertia([0,0,0,0,0,0]); 

finger_thumb.list_links(2).set_mass(0.04); % in kg
finger_thumb.list_links(2).set_inertia([3.1250e-06,1.3562e-05,1.3562e-05,0,0,0]); 

finger_thumb.list_links(3).set_mass(0); % in kg
finger_thumb.list_links(3).set_inertia([0,0,0,0,0,0]); 

finger_thumb.list_links(4).set_mass(0.027); % in kg
finger_thumb.list_links(4).set_inertia([1.6335e-06,6.9007e-06,6.9007e-06,0,0,0]); 

finger_thumb.list_links(5).set_mass(0.016); % in kg
finger_thumb.list_links(5).set_inertia([1.0200e-06,2.2213e-06,2.2213e-06,0,0,0]); 
finger_thumb.update_finger_par_dyn;

%   

finger_little.list_links(1).set_mass(0.03); % in kg
finger_little.list_links(1).set_inertia([4.2725e-06,1.6385e-05,1.4532e-05,0,0,0]); 

finger_little.list_links(2).set_mass(0); % in kg
finger_little.list_links(2).set_inertia([0,0,0,0,0,0]); 

finger_little.list_links(3).set_mass(0.03); % in kg
finger_little.list_links(3).set_inertia([1.8100e-06,1.0000e-05,9.8100e-06,0,0,0]); 

finger_little.list_links(4).set_mass(0.017); % in kg
finger_little.list_links(4).set_inertia([8.6842e-07,2.7257e-06,2.6761e-06,0,0,0]); 

finger_little.list_links(5).set_mass(0.012); % in kg
finger_little.list_links(5).set_inertia([5.3425e-07,1.0530e-06,9.3925e-07,0,0,0]); 
finger_little.update_finger_par_dyn;
% % update dynamic parameters
% finger_thumb.update_finger_par_dyn;
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

finger_index.update_finger(q_I_0);
finger_index.update_M_coupling(q_I_0);
finger_index.M_coupling;


if finger_thumb.nt == 0
    finger_thumb.add_tendon('Flex_1', [1,1,-1,1,1]);
    finger_thumb.add_tendon('Flex_2', [-1,1,1,1,0]);
    finger_thumb.add_tendon('Flex_3', [1,1,-1,0,0]);
    finger_thumb.add_tendon('Flex_4', [-1,1,0,0,0]);
    finger_thumb.add_tendon('Flex_5', [1,0,0,0,0]);
    finger_thumb.add_tendon('Ext_1', [1,-1,-1,-1,-1]);
    finger_thumb.add_tendon('Ext_2', [-1,-1,1,-1,0]);
    finger_thumb.add_tendon('Ext_3', [1,-1,-1,0,0]);
    finger_thumb.add_tendon('Ext_4', [-1,-1,0,0,0]);
    finger_thumb.add_tendon('Ext_5', [1,0,0,0,0]);
end
finger_thumb.update_finger(q_T_0);
finger_thumb.update_M_coupling(q_T_0);
finger_thumb.M_coupling;

if finger_little.nt == 0
    finger_little.add_tendon('Flex_1', [1,1,-1,1,1]);
    finger_little.add_tendon('Flex_2', [-1,1,1,1,0]);
    finger_little.add_tendon('Flex_3', [1,1,-1,0,0]);
    finger_little.add_tendon('Flex_4', [-1,1,0,0,0]);
    finger_little.add_tendon('Flex_5', [1,0,0,0,0]);
    finger_little.add_tendon('Ext_1', [1,-1,-1,-1,-1]);
    finger_little.add_tendon('Ext_2', [-1,-1,1,-1,0]);
    finger_little.add_tendon('Ext_3', [1,-1,-1,0,0]);
    finger_little.add_tendon('Ext_4', [-1,-1,0,0,0]);
    finger_little.add_tendon('Ext_5', [1,0,0,0,0]);
end
finger_little.update_finger(q_L_0);
finger_little.update_M_coupling(q_L_0);
finger_little.M_coupling;

% set joint limits
finger_index.list_joints(1).q_limits = [-20,20]*pi/180; % joints
% finger_index.list_joints(1).qd_limits = [-15,15]*pi/180; % joints
% finger_index.list_joints(1).qdd_limits = [-15,15]*pi/180; % joints
finger_index.list_joints(2).q_limits = [-15,90]*pi/180; % joints
finger_index.list_joints(3).q_limits = [0,90]*pi/180; % 
finger_index.list_joints(4).q_limits = [0,90]*pi/180; % 
finger_index.update_joints_info;


finger_thumb.list_joints(5).q_limits = [-15,90]*pi/180; % 
finger_thumb.list_joints(4).q_limits = [-30,30]*pi/180; % 
finger_thumb.list_joints(3).q_limits = [-12,12]*pi/180; % 
finger_thumb.list_joints(2).q_limits = [0,70]*pi/180; % a
finger_thumb.list_joints(1).q_limits = [-60,60]*pi/180; % abduction joints
finger_thumb.update_joints_info;

finger_little.list_joints(1).q_limits = [0,45]*pi/180; % joints
finger_little.list_joints(2).q_limits = [-20,20]*pi/180; % joints
finger_little.list_joints(3).q_limits = [-15,90]*pi/180; % joints
finger_little.list_joints(4).q_limits = [0,90]*pi/180; % 
finger_little.list_joints(5).q_limits = [0,90]*pi/180; % 
finger_little.update_joints_info;

% add contacts
if finger_index.nc == 0
    for i = finger_index.nl
        finger_index.list_links(i).add_contact([finger_index.list_links(i).Length/2 0 0]');
    end
end
finger_index.update_list_contacts; % update link


% add contacts
if finger_thumb.nc == 0
    for i = finger_thumb.nl
        finger_thumb.list_links(i).add_contact([finger_thumb.list_links(i).Length/2 0 0]');
    end
end
finger_thumb.update_list_contacts; % update link
% 

finger_middle = copy(finger_index);
finger_middle.w_p_base = R*[0.133;0.011;0]; 


finger_ring = copy(finger_index);
finger_ring.w_p_base = R*[0.129;-0.011;0]; 


%% plot 2 fingers

h = figure(2);
set(0,'defaultfigurecolor','w')
set(groot,'defaulttextinterpreter','none');
set(h,'units','normalized','outerposition',[0 0 1 1])
set(h, 'Units', 'centimeters')
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)]);%
set(h, 'Units', 'centimeters')
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

finger_index.update_finger([0,pi/6,pi/6,pi/10]);
finger_index.print_finger('g',5,8)
finger_thumb.update_finger([0,pi/3,0,pi/10,pi/10]);
finger_thumb.print_finger('k',5,8)
finger_middle.update_finger([0,pi/8,pi/10,pi/20])
finger_middle.print_finger('g',5,8)
finger_ring.update_finger([0,0.3,0.1,0.1])
finger_ring.print_finger('g',5,8)
finger_little.update_finger([0.2,0,0.2,0.1,0.1])
finger_little.print_finger('c',5,8)

plot3(0,0,0,'.','Color','k','MarkerSize',50 )

% xlim([0 .2])
% ylim([-0.02 .06])
% zlim([-0.02 .12])
% xticks([0 0.05 0.1 0.15 0.2])
% yticks([-0 0.05 0.1])
% zticks([0 0.05 0.1])
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
grid on

% 
% figure
[X,Y,Z] = sphere(30);
X = 0.04*X + 0.15;
Y = 0.04*Y + 0.02;
Z = 0.04*Z + 0.02;
surf(X,Y,Z,'FaceColor','b','FaceAlpha',1)
axis equal
grid off

return


