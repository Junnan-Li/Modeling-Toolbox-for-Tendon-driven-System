%  create the musculoskeletal hand model with inertia and muscle parameters
%
% data from the literature supplementary documents
%
% references:
%   M. Mirakhorlo et al., "Anatomical parameters for musculoskeletal
%   modeling of the hand and wrist," International Biomechanics, vol. 3,
%   no. 1, pp. 1–10, 2016, doi: 10.1080/23335432.2016.1191373.
%
%   Supplemental materials: https://www.tandfonline.com/doi/suppl/10.1080/23335432.2016.1191373
%
% Junnan Li, 07/2024
clear all
close all
clc

%%

plot_setting = 1;
%% plot landmarkers

data_landmarker = readcell('Mojtaba_model_data_landmarker.csv');

figure(1)
fingers_plot_color = {'r','b','c', 'g', 'k'};
for finger = 1:5
    for i = 2:size(data_landmarker,1)
        if contains(data_landmarker{i,1},num2str(finger)) && contains(data_landmarker{i,1},{'h','b'}) ...
                && isnumeric(data_landmarker{i,3})
            plot3(data_landmarker{i,3}* 0.001,data_landmarker{i,4}* 0.001,data_landmarker{i,5}* 0.001, ...
                '.','MarkerSize',20, 'Color',fingers_plot_color{finger});
            hold on
        end

    end
end
axis equal

%% axes data

data_axes = readcell('Mojtaba_model_data_axes.csv');

figure(1)
fingers_plot_color = {'r','b','c', 'g', 'k'};
for finger = 1:5
    for i = 3:size(data_axes,1)
        if contains(data_axes{i,1},num2str(finger)) && isnumeric(data_axes{i,3})
            plot3([data_axes{i,7}+10*data_axes{i,3};data_axes{i,7} - 10*data_axes{i,3}]* 0.001,...
                [data_axes{i,8}+10*data_axes{i,4};data_axes{i,8} - 10*data_axes{i,4}]* 0.001,...
                [data_axes{i,9}+10*data_axes{i,5};data_axes{i,9} - 10*data_axes{i,5}]* 0.001,...
                '--','LineWidth',2, 'Color',fingers_plot_color{finger});
            hold on
%             quiver3(data_axes{i,7},data_axes{i,8},data_axes{i,9},...
%             5*data_axes{i,3},5*data_axes{i,4},5*data_axes{i,5},...
%             'LineWidth',4,'Color',fingers_plot_color{finger})
%             hold on
        end

    end
end
axis equal
%% cylinder data

data_cylinder = readcell('Mojtaba_model_data_cylinder.csv');

figure(1)
fingers_plot_color = {'r','b','c', 'g', 'k'};
for finger = 1:5
    for i = 3:size(data_cylinder,1)
        if contains(data_cylinder{i,1},num2str(finger)) && isnumeric(data_cylinder{i,5})
            plot3(data_cylinder{i,5}* 0.001,data_cylinder{i,6}* 0.001,data_cylinder{i,7}* 0.001, ...
                '*','MarkerSize',30, 'Color',fingers_plot_color{finger});
            hold on
        end

    end
end
axis equal
%% create middle finger
finger_i = 3;
cylinder_position_middle = [];
cylinder_axis_middle = [];
for i = 3:size(data_cylinder,1)
    if contains(data_cylinder{i,1},num2str(finger_i)) ...
            && isnumeric(data_cylinder{i,3})
        cylinder_position_middle = [cylinder_position_middle; ...
        [data_cylinder{i,5},data_cylinder{i,6},data_cylinder{i,7}]*0.001];
        cylinder_axis_middle = [cylinder_axis_middle; ...
        [data_cylinder{i,2},data_cylinder{i,3},data_cylinder{i,4}]];
    end
end
length_MP3 = norm(cylinder_position_middle(2,:)'- cylinder_position_middle(1,:)',2);
length_PP3 = norm(cylinder_position_middle(3,:)'- cylinder_position_middle(2,:)',2);
length_DP3 = 0.02; % set DP length is 0.02 m
finger_dimension_3 = [0,length_MP3,length_PP3,length_DP3]; % in meter
finger_m = Finger('Middle', 'type','R_RRRR', 'l_links',finger_dimension_3);
mdh_default_struct = finger_m.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
% mdh_matrix(2,4) = 1;
finger_m.set_mdh_parameters(mdh_matrix);

% set base information
p_32 = cylinder_position_middle(2,1:2)'- cylinder_position_middle(1,1:2)';
eul_base = -acos(([1,0]*p_32)/norm(p_32));
finger_m.set_base(cylinder_position_middle(1,:)',euler2R_XYZ([0,0,eul_base]));
finger_m.update_finger(zeros(finger_m.nj,1));

plot_par = finger_m.plot_parameter_init;
finger_m.plot_finger(plot_par);
return
%% create index finger
% transfer to finger length
finger_i = 2;
landmarker_position_index = [];
for i = 2:size(data_landmarker,1)
    if contains(data_landmarker{i,1},num2str(finger_i)) && contains(data_landmarker{i,1},{'h','b'}) ...
            && isnumeric(data_landmarker{i,3})
        landmarker_position_index = [landmarker_position_index; ...
        [data_landmarker{i,3},data_landmarker{i,4},data_landmarker{i,5}]*0.001];
    end
end
length_MP2 = norm(landmarker_position_index(2,:)'- landmarker_position_index(3,:)',2);
length_PP2 = norm(landmarker_position_index(3,:)'- landmarker_position_index(4,:)',2);
length_DP2 = length_DP3; % Due to the lack of data in reference
finger_dimension_2 = [0,length_MP2,length_PP2,length_DP2]; % in meter
finger_i = Finger('Index', 'type','R_RRRR', 'l_links',finger_dimension_3);
mdh_default_struct = finger_i.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
finger_i.set_mdh_parameters(mdh_matrix);

% set base
p_32 = landmarker_position_index(3,1:2)'- landmarker_position_index(2,1:2)';
eul_base = -acos(([1,0]*p_32)/norm(p_32));
finger_i.set_base(landmarker_position_index(2,:)',euler2R_XYZ([0,0,eul_base]));
finger_i.update_finger(zeros(finger_i.nj,1));

finger_i.plot_finger(plot_par);

return
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