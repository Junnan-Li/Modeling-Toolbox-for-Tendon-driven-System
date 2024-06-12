%   example script for creating an example of finger

%% create finger with tendons

clear all 
close all
clc


%% load urdf model

finger_urdf = importrobot('.\examples\hand_one\URDF_FINGERS_4DOF-torsion_modified.urdf');
% finger_urdf = importrobot('..\Handgroup_finger\finger_files\Finger_V2-URDF-DH_Parameter\A-S_FINGERS_4DOF-torsion_test.SLDASM\urdf\A-S_FINGERS_4DOF-torsion_test.SLDASM.urdf');
finger_urdf.DataFormat = "column";
% 

%%
% r = rateControl(100);
% figure
% xlim([-0.05, 0.05])
% for i = 1:length(out_q.time)
%     finger_urdf.show(out_q.signals.values(i,:)','Frame','off','PreservePlot',0);
%     hold on
%     xlim([-0.1, 0.1])
%     ylim([-0.1, 0.1])
%     zlim([-0.1, 0.1])
%     grid off
%     drawnow;
%     exportgraphics(gcf,'testAnimated.gif','Append',true);
% %     waitfor(r);
% end
%%
q = [1,1,1,1]';
finger_urdf.show(q,'Frame','off','PreservePlot',0);
% return
%%

% config = homeConfiguration(finger);
% config = [pi/4 pi/4 pi/4 pi/8]';
% config = [0 0 0 0]';

% global finger_handone
finger_handone = init_from_rst(finger_urdf);
mdh_ori = finger_handone.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_ori,1);
mdh_matrix(5,2) = 0.014;

finger_handone.set_mdh_parameters(mdh_matrix);
% finger_handone.w_p_base = 4*zeros(3,1);
% finger_handone.w_R_base = euler2R_XYZ([pi,0,0]);
% finger_handone.update_finger(config);

% load rst model from finger class
rst_model_r = finger_handone.rst_model;

% finger_handone.compile_functions

%% simulation setting

q_init = [0,0,0,0]';
qD_init = [0,0,0,0]';


% joint stiffness 
q_stiff_init = [0,0,0,0]';
S_j = 1e-4 * [1,1,1,1]';
D_j = 1e-4 * [1,1,1,1]';

% coupling tendon stiffness
k_coupling = 1e-1;
d_coupling = 1e-5;



return

%% show model
% figure(1)
% subplot(1,2,1)
% show(finger_urdf,config,'visuals','on','collision','off','Frames', 'off');
% hold on
% % plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','r');
% 
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
% [caz,cel] = view;
% subplot(1,2,2)
% finger_handone.print_finger('r',5)
% hold on
% grid on
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
% view(caz,cel)


%% add tendons

if finger_r.nt == 0
    finger_r.add_tendon('Flex_1', [1,1,1,1]);
    finger_r.add_tendon('Flex_2', [1,1,1,0]);
    finger_r.add_tendon('Flex_3', [1,1,0,0]);
    finger_r.add_tendon('Flex_4', [1,0,0,0]);
    finger_r.add_tendon('Ext_1', [-1,-1,-1,-1]);
    finger_r.add_tendon('Ext_2', [-1,-1,-1,0]);
    finger_r.add_tendon('Ext_3', [-1,-1,0,0]);
    finger_r.add_tendon('Ext_4', [-1,0,0,0]);
end

% finger_r.set_tendon_par_MA_poly3(1,1,[0,0,0.01,0.03]);
% finger_r.set_tendon_par_MA_poly3(3,1,[0,0,0.01,0.3]);
% finger_r.set_tendon_par_MA_poly3(3,2,[0,0,0.01,0.035]);
% finger_r.set_tendon_par_MA_poly3(4,1,[0,0,0.001,0.02]);

finger_r.update_finger(q_0);
finger_r.update_M_coupling(q_0);
finger_r.M_coupling;


% set joint limits
finger_r.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints

finger_r.update_joints_info;


% add contacts
if finger_r.nc == 0
    for i = 1:finger_r.nl
        finger_r.list_links(i).add_contact([finger_r.list_links(i).Length/2 0 0]');
    end
end
finger_r.update_list_contacts; % update link


return
%





