%  create the musculoskeletal hand model with inertia and muscle parameters
%  
% data from the literature supplementary documents
% 
% references:
%   M. Mirakhorlo et al., "Anatomical parameters for musculoskeletal 
%   modeling of the hand and wrist," International Biomechanics, vol. 3, 
%   no. 1, pp. 1–10, 2016, doi: 10.1080/23335432.2016.1191373.
% 
% Junnan Li, 07/2024
clear all
close all
clc


%% 
hand_data = struct();

% data in meter
hand_data.index.axes.MCP_flex = [47.3;-92.88;0.29]*0.001;
hand_data.index.axes.MCP_flex_axis = [-0.99;-0.11;-0.09]*0.001;
hand_data.index.axes.MCP_abd = [47.24;-93.18;0.05]*0.001;
hand_data.index.axes.MCP_abd_axis = [0;0.42;0.91]*0.001;
hand_data.index.axes.PIP_flex = [52.9;-134.3;2.76]*0.001;
hand_data.index.axes.PIP_flex_axis = [-0.99;-0.11;-0.07]*0.001;
hand_data.index.axes.DIP_flex = [49.47;-155.97;-0.29]*0.001;
hand_data.index.axes.DIP_flex_axis = [-0.96;-0.17;-0.21]*0.001;

hand_data.middle.axes.MCP_flex = [31.83;-92.51;0.73]*0.001;
hand_data.middle.axes.MCP_flex_axis = [-0.99;0.06;-0.08]*0.001;
hand_data.middle.axes.MCP_abd = [28.71;-95.88;-3.01]*0.001;
hand_data.middle.axes.MCP_abd_axis = [-0.02;-0.20;-0.98]*0.001;
hand_data.middle.axes.PIP_flex = [42.74;-137.97;-6.01]*0.001;
hand_data.middle.axes.PIP_flex_axis = [-0.99;0.10;-0.12]*0.001;
hand_data.middle.axes.DIP_flex = [33.06;-163.59;-19.28]*0.001;
hand_data.middle.axes.DIP_flex_axis = [-0.97;0.19;-0.14]*0.001;

hand_data.ring.axes.MCP_flex = [12.12;-86.07;-1.98]*0.001;
hand_data.ring.axes.MCP_flex_axis = [-0.97;0.22;-0.11]*0.001;
hand_data.ring.axes.MCP_abd = [7.05;-85.94;-2.27]*0.001;
hand_data.ring.axes.MCP_abd_axis = [-0.26;-0.22;-0.94]*0.001;
hand_data.ring.axes.PIP_flex = [-2.65;-125.60;1.48]*0.001;
hand_data.ring.axes.PIP_flex_axis = [-0.88;0.36;-0.32]*0.001;
hand_data.ring.axes.DIP_flex = [-2.50;-152.19;-6.44]*0.001;
hand_data.ring.axes.DIP_flex_axis = [-0.92;0.18;-0.35]*0.001;

hand_data.little.axes.MCP_flex = [-2.44;-79.11;-3.20]*0.001;
hand_data.little.axes.MCP_flex_axis = [-0.91;0.33;-0.24]*0.001;
hand_data.little.axes.MCP_abd = [-6.01;-79.66;1.68]*0.001;
hand_data.little.axes.MCP_abd_axis = [-0.01;-0.31;-0.95]*0.001;
hand_data.little.axes.PIP_flex = [-13.57;-112.38;0.66]*0.001;
hand_data.little.axes.PIP_flex_axis = [-0.84;0.28;-0.47]*0.001;
hand_data.little.axes.DIP_flex = [-17.20;-129.98;-9.24]*0.001;
hand_data.little.axes.DIP_flex_axis = [-0.91;0.28;-0.30]*0.001;

hand_data.thumb.axes.CMC_flex = [48.54;-37.82;-6.63]*0.001;
hand_data.thumb.axes.CMC_flex_axis = [-0.27;0.64;-0.72]*0.001;
hand_data.thumb.axes.CMC_abd = [49.16;-39.46;-13.15]*0.001;
hand_data.thumb.axes.CMC_abd_axis = [-0.78;-0.61;-0.13]*0.001;
hand_data.thumb.axes.MCP_flex = [68.92;-64.16;-3.54]*0.001;
hand_data.thumb.axes.MCP_flex_axis = [-0.67;-0.07;0.74]*0.001;
hand_data.thumb.axes.IP_flex = [88.26;-89.46;4.58]*0.001;
hand_data.thumb.axes.IP_flex_axis = [-0.36;-0.25;0.90]*0.001;

% plot the joint axes points from literature

Fingers_list = {'index','middle', 'ring','little', 'thumb'};
fingers_plot_color = {'r','b','c', 'g'};
figure(1)
for i = 1:4
eval(strcat('finger_i = hand_data.', Fingers_list{i},'.axes')); 
quiver3(finger_i.MCP_flex(1),finger_i.MCP_flex(2),finger_i.MCP_flex(3),...
    3*finger_i.MCP_flex_axis(1),3*finger_i.MCP_flex_axis(2),3*finger_i.MCP_flex_axis(3),...
    'LineWidth',4,'Color',fingers_plot_color{i})
hold on
quiver3(finger_i.MCP_abd(1),finger_i.MCP_abd(2),finger_i.MCP_abd(3),...
    3*finger_i.MCP_abd_axis(1),3*finger_i.MCP_abd_axis(2),3*finger_i.MCP_abd_axis(3),...
    'LineWidth',4,'Color',fingers_plot_color{i})
hold on
quiver3(finger_i.PIP_flex(1),finger_i.PIP_flex(2),finger_i.PIP_flex(3),...
    3*finger_i.PIP_flex_axis(1),3*finger_i.PIP_flex_axis(2),3*finger_i.PIP_flex_axis(3),...
    'LineWidth',4,'Color',fingers_plot_color{i})
hold on
quiver3(finger_i.DIP_flex(1),finger_i.DIP_flex(2),finger_i.DIP_flex(3),...
    3*finger_i.DIP_flex_axis(1),3*finger_i.DIP_flex_axis(2),3*finger_i.DIP_flex_axis(3),...
    'LineWidth',4,'Color',fingers_plot_color{i})
end


axis equal
%% create index finger


% transfer to finger length



finger_dimension = [1,5,3,2]; % in meter
finger_r = Finger('Index', 'type','R_RRRR', 'l_links',finger_dimension); 
% finger_r = Finger('hand_one','mdh',)
mdh_default_struct = finger_r.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
% mdh_matrix(2,4) = 1;
finger_r.set_mdh_parameters(mdh_matrix);

%% basic settings
% set base position and orientation
finger_r.set_base(4*rand(3,1),euler2R_XYZ(rand(1,3)));

% init joint configurations
q_0 = [pi/4;pi/4;1;1];

% udpate finger with given joint configurations
finger_r.update_finger(q_0);
finger_r.update_rst_model;

% set dynamic parameters

finger_r.list_links(1).set_mass(0.04); % in kg
finger_r.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(2).set_mass(0.03); % in kg
finger_r.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(3).set_mass(0.02); % in kg
finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(4).set_mass(0.02); % in kg
finger_r.list_links(4).set_inertia([0.5,0.2,0.2,0,0,0]); 
% update dynamic parameters
finger_r.update_finger_par_dyn;
finger_r.update_rst_model;