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
xlabel('x')
ylabel('y')
zlabel('z')
%% create middle finger
finger_m = create_Mojtaba_finger_from_csv(3, 'Middle');
finger_i = create_Mojtaba_finger_from_csv(2, 'Index');
finger_r = create_Mojtaba_finger_from_csv(4, 'Ring');
finger_l = create_Mojtaba_finger_from_csv(5, 'Little');

thumb = create_Mojtaba_thumb_from_csv('Thumb');
plot_par = finger_m.plot_parameter_init;

finger_m.plot_finger(plot_par)
finger_i.plot_finger(plot_par)
finger_r.plot_finger(plot_par)
finger_l.plot_finger(plot_par)
thumb.plot_finger(plot_par)

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