%  create index finger of hand prototype

%% create finger with tendons

clear all 
close all
clc

%%
PTI_urdf = importrobot('.\examples\hand_one\PalmThumbIndex\Palm_Thumb_Index_modified.urdf');
% finger_urdf = importrobot('..\Handgroup_finger\finger_files\Finger_V2-URDF-DH_Parameter\A-S_FINGERS_4DOF-torsion_test.SLDASM\urdf\A-S_FINGERS_4DOF-torsion_test.SLDASM.urdf');
PTI_urdf.DataFormat = "column";
% 


%% load urdf model

finger_urdf = importrobot('.\examples\hand_one\URDF_FINGERS_4DOF-torsion_modified.urdf');
finger_urdf.DataFormat = "column";

%% seperated finger model

finger_index = create_handone_finger_from_urdf(finger_urdf,'index');
finger_index.set_base([0.103,0.02,0]', euler2R_XYZ([3-180,-5.8,-5]*pi/180));

finger_middle = create_handone_finger_from_urdf(finger_urdf,'middle');
finger_middle.set_base([0.105,-0.004,0]', euler2R_XYZ([pi,0,0]));

finger_ring = create_handone_finger_from_urdf(finger_urdf,'ring');
finger_ring.set_base([0.106,-0.027,0]', euler2R_XYZ([3-180,-5.8,5]*pi/180));

finger_little = create_handone_finger_from_urdf(finger_urdf,'little');
finger_little.set_base([0.1,-0.05,0]', euler2R_XYZ([5-180,-8.6,7]*pi/180));

finger_thumb = create_handone_finger_from_urdf(finger_urdf,'thumb');
finger_thumb.set_base([0.06,0.047,-0.024]', eul2rotm([48,48,80]*pi/180,'ZYX'));
% finger_thumb.set_base([0.06,0.047,-0.024]', eul2rotm([48,122,78]*pi/180,'XYZ'));
finger_list = {finger_index,finger_middle,finger_ring,finger_little,finger_thumb};

% set the joint limits

for i = 1:length(finger_list)
    finger_list{i}.set_onejoint_limits_q(1,[-15,15]*pi/180);
    finger_list{i}.set_onejoint_limits_q(2,[0,90]*pi/180);
    finger_list{i}.set_onejoint_limits_q(3,[0,100]*pi/180);
    finger_list{i}.set_onejoint_limits_q(4,[0,90]*pi/180);
end


%% plot
% plot_par = finger_index.plot_parameter_init;
% plot_par.axis_len = 0.01;
% plot_par.markersize = 5;
% q = [0,0.2,0.3,0]';
% q = zeros(4,1);
% figure(1)
% for i = 1:length(finger_list)
%     finger_list{i}.update_finger(q);
%     finger_list{i}.plot_finger(plot_par);
%     finger_list{i}.plot_com(plot_par);
%     axis equal
% end

%% Hand object


mdh_wrist = zeros(3,4);
% forearm length (base to first frame)
mdh_wrist(1,2) = 0.4; % value doesn't matter
mdh_wrist(2,1) = pi/2;
mdh_wrist(3,1) = -pi/2;
mdh_struct = mdh_matrix_to_struct(mdh_wrist, 1);
wrist = Finger('wrist', 'mdh' ,mdh_struct );
wrist.set_base(ones(3,1), euler2R_XYZ([0,0,0]));

% set base information

wrist.list_links(1).set_mass(0);
wrist.list_links(1).set_com([0,0,0]);
wrist.list_links(1).set_inertia([zeros(3,1);zeros(3,1)]);

wrist.list_links(2).set_mass(0.07);
wrist.list_links(2).set_com([0.2,0,0]);
wrist.list_links(2).set_inertia([rand(3,1);zeros(3,1)]);




q_init = [0.2,0.2]';
wrist.update_finger(q_init);
% wrist.plot_finger(plot_par);

handone = Hand('handone');
handone.add_base(wrist);
handone.add_finger(finger_index);
handone.add_finger(finger_middle);
handone.add_finger(finger_ring);
handone.add_finger(finger_little);
handone.add_finger(finger_thumb);

q = [0;0;zeros(handone.nj-2,1)];
% q = zeros(handone.nj,1);
handone.set_joint_limits_on;
handone.update_hand(q);
plot_par = handone.plot_parameter_init;
plot_par.axis_len = 0.02;
figure(2)
handone.plot_hand(plot_par)
axis equal


hand_rst = handone.update_rst_model;
% hand_rst.show(q,"Frames","on");

J = handone.Jacobian_geom_w_one_finger(1,q);

handone.update_sim_mdh
%%
w_T_all_links = handone.get_w_T_links_inhand;
sim_plot_hand(w_T_all_links,handone.sim_n_links,handone.sim_q_index)
