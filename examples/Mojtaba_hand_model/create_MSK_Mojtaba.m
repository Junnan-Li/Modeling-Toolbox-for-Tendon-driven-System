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

% figure(1)
% fingers_plot_color = {'r','b','c', 'g', 'k'};
% for finger = 1:5
%     for i = 3:size(data_cylinder,1)
%         if contains(data_cylinder{i,1},num2str(finger)) && isnumeric(data_cylinder{i,5})
%             plot3(data_cylinder{i,5}* 0.001,data_cylinder{i,6}* 0.001,data_cylinder{i,7}* 0.001, ...
%                 '*','MarkerSize',30, 'Color',fingers_plot_color{finger});
%             hold on
%         end
% 
%     end
% end
% axis equal
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

% udpate finger with given joint configurations
% finger_r.update_finger(q_0);
% finger_r.update_rst_model;

%% set dynamic parameters
% data source: Mirakhorlo et al., “Anatomical Parameters for Musculoskeletal Modeling of the Hand and Wrist.”

data_inertia = readcell('Mojtaba_model_data_inertia.csv');

for finger = 1:5
    for i = 2:size(data_inertia,1)
        if contains(data_inertia{i,1},num2str(finger)) && isnumeric(data_inertia{i,3}) ...
                && isnumeric(data_inertia{i,3})
            plot3(data_inertia{i,3}*1e-3,data_inertia{i,4}*1e-3,data_inertia{i,5}*1e-3,...
                "x",'MarkerSize',15, 'Color',"m");
            hold on
        end

    end
end

finger_m = set_Mojtaba_finger_inertia_from_csv(finger_m, 3);
finger_i = set_Mojtaba_finger_inertia_from_csv(finger_i, 3);
finger_r = set_Mojtaba_finger_inertia_from_csv(finger_r, 4);
finger_l = set_Mojtaba_finger_inertia_from_csv(finger_l, 5);
thumb = set_Mojtaba_finger_inertia_from_csv(thumb, 5);

finger_m.plot_com(plot_par)
finger_i.plot_com(plot_par)
finger_r.plot_com(plot_par)
finger_l.plot_com(plot_par)
thumb.plot_com(plot_par)

%% check the wrist and forearm landmarkers and reset the finger base


figure(1)
% plot forearm landmarkers
for i = 2:5
    plot3(data_landmarker{i,3}* 0.001,data_landmarker{i,4}* 0.001,data_landmarker{i,5}* 0.001, ...
        'o','MarkerSize',20, 'Color','b');
    hold on
end

for i = 3:6
    plot3([data_axes{i,7}+20*data_axes{i,3};data_axes{i,7} - 20*data_axes{i,3}]* 0.001,...
                [data_axes{i,8}+20*data_axes{i,4};data_axes{i,8} - 20*data_axes{i,4}]* 0.001,...
                [data_axes{i,9}+20*data_axes{i,5};data_axes{i,9} - 20*data_axes{i,5}]* 0.001,...
                '--','LineWidth',2, 'Color','b');
    hold on
end

position_new_base = ([data_landmarker{4,3:5}] + [data_landmarker{5,3:5}])'/2 * 0.001;
finger_m.set_base_p(finger_m.get_base_p - position_new_base);
finger_i.set_base_p(finger_i.get_base_p - position_new_base);
finger_r.set_base_p(finger_r.get_base_p - position_new_base);
finger_l.set_base_p(finger_l.get_base_p - position_new_base);
thumb.set_base_p(thumb.get_base_p - position_new_base);

figure(2)
finger_m.plot_finger(plot_par)
finger_i.plot_finger(plot_par)
finger_r.plot_finger(plot_par)
finger_l.plot_finger(plot_par)
thumb.plot_finger(plot_par)

finger_m.plot_com(plot_par)
finger_i.plot_com(plot_par)
finger_r.plot_com(plot_par)
finger_l.plot_com(plot_par)
thumb.plot_com(plot_par)
axis equal

%% create forearm
% version 1: only wrist flexion and abduction
% the palm mass is set
% the center of mass is set as the middle point of mc bases
position_EL_EM = ([data_landmarker{1,3:5}] + [data_landmarker{2,3:5}])'/2 * 0.001;
position_US_RS = ([data_landmarker{3,3:5}] + [data_landmarker{4,3:5}])'/2 * 0.001;
length_forearm = norm(position_US_RS-position_EL_EM);

mdh_parameter1 = [0,0,0,0;...
                pi/2,0,0,0;...
                pi/2,0,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter1, 1);
wrist = Finger('wrist', 'mdh',mdh_struct );
wrist.set_base([0,length_forearm,0]',euler2R_XYZ([0,0,pi/2]));


% set wrist dynamic
% center of mass is set as base of MCs
MC_palm = [];
for finger = 2:5
    for i = 2:size(data_landmarker,1)
        if contains(data_landmarker{i,1},num2str(finger)) && contains(data_landmarker{i,1},{'b'}) ...
                && isnumeric(data_landmarker{i,3})
            MC_palm = [MC_palm; [data_landmarker{i,3},data_landmarker{i,4},data_landmarker{i,5}]*1e-3]; 
        end
    end
end
com_palm = mean(MC_palm,1)' - position_new_base;
wrist.list_links(2).set_mass(0.332);
wrist.list_links(2).set_com(reshape(com_palm,3,1));

wrist.plot_finger(plot_par)
wrist.plot_com(plot_par)



