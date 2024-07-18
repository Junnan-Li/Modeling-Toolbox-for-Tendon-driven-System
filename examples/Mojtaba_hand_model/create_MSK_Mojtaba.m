%  create the musculoskeletal hand model with inertia and muscle parameters
%
% data from the literature supplementary documents
%
%   wrist:      
%   fingers:
%   dof sequence:       wrist radial deviation, flexion, ...
%                       index MCP flexion, MCP abduction, PIP, DIP, ...
%                       middle,..., ring, ..., little, ..., thumb
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


axis equal

%% axes data

data_axes = readcell('Mojtaba_model_data_axes.csv');


%% muscle path data
data_muscle = readcell('Mojtaba_model_data_muscle_path.csv');

% return

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
% xlabel('x')
% ylabel('y')
% zlabel('z')


%% create fingers
finger_m = create_Mojtaba_finger_from_csv(3, 'Middle');
finger_i = create_Mojtaba_finger_from_csv(2, 'Index');
finger_r = create_Mojtaba_finger_from_csv(4, 'Ring');
finger_l = create_Mojtaba_finger_from_csv(5, 'Little');

thumb = create_Mojtaba_thumb_from_csv('Thumb');



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

% finger_m.plot_com(plot_par)
% finger_i.plot_com(plot_par)
% finger_r.plot_com(plot_par)
% finger_l.plot_com(plot_par)
% thumb.plot_com(plot_par)


%% inverse kinematics

% middle finger
x_middle = [data_landmarker{31,3:5}]' *  1e-3;
[q_mid, ~, x_res,phi_x,iter] = finger_m.invkin_trans_numeric(x_middle);


% ring finger
x_ring = [data_landmarker{40,3:5}]' *  1e-3;
[q_ring, ~, x_res,phi_x,iter] = finger_r.invkin_trans_numeric(x_ring);

% little finger
x_little = [data_landmarker{49,3:5}]' *  1e-3;
[q_little, ~, x_res,phi_x,iter] = finger_l.invkin_trans_numeric(x_little);



% thumb finger
x_thumb = [data_landmarker{13,3:5}]' *  1e-3;
thumb.update_finger(rand(thumb.nj,1));
[q_thumb, ~, x_res,phi_x,iter] = thumb.invkin_trans_numeric(x_thumb);



%%%%%%%%%%%%%%%%%
% result of ik to set muscle and viapoints
q_thumb = [-0.13,-0.25,0.05,0.2]';
q_index = [-0.12,0,0.22,0.3]';
q_mid = [0.0878;0.0151;0.4431;0.2774];
q_ring = [-0.1388;0.1142;0.4029;0.2329];
q_little = [-0.1423;0.0019;0.4834;0.2594];

finger_i.update_finger(q_index)
finger_m.update_finger(q_mid)
finger_r.update_finger(q_ring)
finger_l.update_finger(q_little)
thumb.update_finger(q_thumb)
%
plot_par = finger_m.plot_parameter_init;
finger_m.plot_finger(plot_par)
finger_i.plot_finger(plot_par)
finger_r.plot_finger(plot_par)
finger_l.plot_finger(plot_par)
thumb.plot_finger(plot_par)
plot_Mojtaba_landmarkers()
%% check the wrist and forearm landmarkers and reset the finger base
%%%%%%%%%%%
% figure(1)
% % plot forearm landmarkers
% for i = 2:5
%     plot3(data_landmarker{i,3}* 0.001,data_landmarker{i,4}* 0.001,data_landmarker{i,5}* 0.001, ...
%         'o','MarkerSize',20, 'Color','b');
%     hold on
% end
% for i = 3:6
%     plot3([data_axes{i,7}+20*data_axes{i,3};data_axes{i,7} - 20*data_axes{i,3}]* 0.001,...
%                 [data_axes{i,8}+20*data_axes{i,4};data_axes{i,8} - 20*data_axes{i,4}]* 0.001,...
%                 [data_axes{i,9}+20*data_axes{i,5};data_axes{i,9} - 20*data_axes{i,5}]* 0.001,...
%                 '--','LineWidth',2, 'Color','b');
%     hold on
% end
%%%%%%%%%%%%%%
position_new_base = ([data_landmarker{4,3:5}] + [data_landmarker{5,3:5}])'/2 * 0.001;
%%%%%%%%%%
% R_new_base = euler2R_XYZ([0,0,pi/2]);
% 
% finger_m.set_base(R_new_base*(finger_m.get_base_p - position_new_base),R_new_base*finger_m.get_base_R);
% finger_i.set_base(R_new_base*(finger_i.get_base_p - position_new_base),R_new_base*finger_i.get_base_R);
% finger_r.set_base(R_new_base*(finger_r.get_base_p - position_new_base),R_new_base*finger_r.get_base_R);
% finger_l.set_base(R_new_base*(finger_l.get_base_p - position_new_base),R_new_base*finger_l.get_base_R);
% thumb.set_base(R_new_base*(thumb.get_base_p - position_new_base),R_new_base*thumb.get_base_R);
%%%%%%%%%%%%%%%%

% figure(2)
% finger_m.plot_finger(plot_par)
% finger_i.plot_finger(plot_par)
% finger_r.plot_finger(plot_par)
% finger_l.plot_finger(plot_par)
% thumb.plot_finger(plot_par)
% 
% finger_m.plot_com(plot_par)
% finger_i.plot_com(plot_par)
% finger_r.plot_com(plot_par)
% finger_l.plot_com(plot_par)
% thumb.plot_com(plot_par)
% axis equal

% create forearm
% version 1: only wrist flexion and abduction
% the palm mass is set
% the center of mass is set as the middle point of mc bases
position_EL_EM = ([data_landmarker{2,3:5}] + [data_landmarker{3,3:5}])'/2 * 0.001;
position_US_RS = ([data_landmarker{4,3:5}] + [data_landmarker{5,3:5}])'/2 * 0.001;
length_forearm = norm(position_US_RS-position_EL_EM);

mdh_parameter1 = [0,0,0,0;...
                pi/2,0,0,0;...
                -pi/2,0,0,0];
mdh_struct = mdh_matrix_to_struct(mdh_parameter1, 1);
wrist = Finger('wrist', 'mdh',mdh_struct );
wrist.set_base(position_US_RS,euler2R_XYZ([0,0,pi/2]));


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

w_T_all = wrist.get_T_all_links;
wrist.base.set_mass(1);
wrist.base.set_com(position_EL_EM-position_US_RS);
wrist.list_links(1).set_mass(0);
wrist.list_links(2).set_mass(0.332);
wrist.list_links(2).set_com(reshape(w_T_all(1:3,1:3)'*com_palm,3,1));
wrist.update_finger_par_dyn

wrist.plot_finger(plot_par)
wrist.plot_com(plot_par)

%%

finger_m.set_base(w_T_all(1:3,1:3,end)'*(finger_m.w_p_base-w_T_all(1:3,4,end)),w_T_all(1:3,1:3,end)'*finger_m.w_R_base);
finger_i.set_base(w_T_all(1:3,1:3,end)'*(finger_i.w_p_base-w_T_all(1:3,4,end)),w_T_all(1:3,1:3,end)'*finger_i.w_R_base);
finger_r.set_base(w_T_all(1:3,1:3,end)'*(finger_r.w_p_base-w_T_all(1:3,4,end)),w_T_all(1:3,1:3,end)'*finger_r.w_R_base);
finger_l.set_base(w_T_all(1:3,1:3,end)'*(finger_l.w_p_base-w_T_all(1:3,4,end)),w_T_all(1:3,1:3,end)'*finger_l.w_R_base);
thumb.set_base(w_T_all(1:3,1:3,end)'*(thumb.w_p_base-w_T_all(1:3,4,end)),w_T_all(1:3,1:3,end)'*thumb.w_R_base);


Mojtaba_hand = Hand('Mojtaba_hand');
Mojtaba_hand.add_base(wrist);
Mojtaba_hand.add_finger(thumb);
Mojtaba_hand.add_finger(finger_i);
Mojtaba_hand.add_finger(finger_m);
Mojtaba_hand.add_finger(finger_r);
Mojtaba_hand.add_finger(finger_l);


rst_model = Mojtaba_hand.update_rst_model;

% update finger joint limits 
for i = 1:5
    Mojtaba_hand.list_fingers(i).set_onejoint_limits_q(1,[0,100]*pi/180);
    Mojtaba_hand.list_fingers(i).set_onejoint_limits_q(2,[-10,10]*pi/180);
    Mojtaba_hand.list_fingers(i).set_onejoint_limits_q(3,[0,90]*pi/180);
    Mojtaba_hand.list_fingers(i).set_onejoint_limits_q(4,[0,90]*pi/180);
end
% Mojtaba_hand.base(1).set_onejoint_limits_q(1,[0,100]*pi/180);
% Mojtaba_hand.base(1).set_onejoint_limits_q(1,[0,100]*pi/180);

Mojtaba_hand.update_joint_limits;
Mojtaba_hand.set_joint_limits_on;

figure(3)

q0 = [0;0;q_thumb;q_index;q_mid;q_ring;q_little];
Mojtaba_hand.update_hand(q0);
Mojtaba_hand.update_hand_par_dyn;

plot_hand_par = Mojtaba_hand.plot_parameter_init;
plot_hand_par.markersize = 10;
plot_hand_par.markercolor = 'k';
Mojtaba_hand.plot_hand(plot_hand_par);
Mojtaba_hand.plot_hand_com(plot_hand_par);
plot_Mojtaba_landmarkers
axis equal
% rst_model.show(q0,'Frames','on')
Mojtaba_hand.plot_hand_viapoints(plot_hand_par)

plot_Mojtaba_muscle_viapoint({'FDS'})

%% add viapoints and muscles
% add viapoint to the links


Seg_linK_mapping = {'Radius', Mojtaba_hand.base.base;...
                    'Humerus', Mojtaba_hand.base.base;...
                    'Ulna', Mojtaba_hand.base.base;...
                    'MC1', Mojtaba_hand.list_fingers(1).base;...
                    'MC2', Mojtaba_hand.list_fingers(2).base;...
                    'MC3', Mojtaba_hand.list_fingers(3).base;...
                    'MC4', Mojtaba_hand.list_fingers(4).base;...
                    'MC5', Mojtaba_hand.list_fingers(5).base;...
                    'MP1', Mojtaba_hand.list_fingers(1).list_links(2);...
                    'MP2', Mojtaba_hand.list_fingers(2).list_links(2);...
                    'MP3', Mojtaba_hand.list_fingers(3).list_links(2);...
                    'MP4', Mojtaba_hand.list_fingers(4).list_links(2);...
                    'MP5', Mojtaba_hand.list_fingers(5).list_links(2);...
                    'PP1', Mojtaba_hand.list_fingers(1).list_links(3);...
                    'PP2', Mojtaba_hand.list_fingers(2).list_links(3);...
                    'PP3', Mojtaba_hand.list_fingers(3).list_links(3);...
                    'PP4', Mojtaba_hand.list_fingers(4).list_links(3);...
                    'PP5', Mojtaba_hand.list_fingers(5).list_links(3);...
                    'DP1', Mojtaba_hand.list_fingers(1).list_links(4);...
                    'DP2', Mojtaba_hand.list_fingers(2).list_links(4);...
                    'DP3', Mojtaba_hand.list_fingers(3).list_links(4);...
                    'DP4', Mojtaba_hand.list_fingers(4).list_links(4);...
                    'DP5', Mojtaba_hand.list_fingers(5).list_links(4);...
                    };

add_Mojtaba_muscle_to_model(Mojtaba_hand, Seg_linK_mapping) % 

Mojtaba_hand.update_list_viapoints;
% 
% %
plot_hand_par = Mojtaba_hand.plot_parameter_init;
plot_hand_par.muscle_linewidth = 2;
plot_hand_par.muscle_linecolor = 'g';
plot_hand_par.muscle_markersize = 10;
plot_hand_par.axis_len = 0.02;
plot_hand_par.viapoint_marker = '.';
plot_hand_par.viapoint_markersize = 10;
plot_hand_par.viapoint_markercolor = 'b';
figure(5)
Mojtaba_hand.plot_hand(plot_hand_par)
axis equal
Mojtaba_hand.plot_hand_viapoints(plot_hand_par)
% plot_Mojtaba_muscle_viapoint()
Mojtaba_hand.plot_hand_muscles(plot_hand_par)


% plot_Mojtaba_axes({'3','2','4'});

%%
% % % q = sym('q',[1 finger_m.nj])
% % % 
% % % mdh_sym = sym(mdh_struct_to_matrix(finger_m.mdh_ori,1))
% % % f_sym = create_Mojtaba_finger_from_csv(3, 'sym');
% % % f_sym.set_mdh_parameters(mdh_sym);  
% % % f_sym.update_finger(q)
% % % f_sym.set_base(sym(f_sym.get_base_p),sym(f_sym.get_base_R))
% % % f_sym.get_p_all_links