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

finger_index = create_handone_finger_from_urdf(finger_urdf,'index');
finger_mid = create_handone_finger_from_urdf(finger_urdf,'mid');
finger_ring = create_handone_finger_from_urdf(finger_urdf,'ring');
finger_little = create_handone_finger_from_urdf(finger_urdf,'little');
finger_thumb = create_handone_finger_from_urdf(finger_urdf,'thumb');



return
%%

handone = Hand('handone');
handone.add_base(wrist);
handone.add_finger(finger_index);
handone.add_finger(finger_middle);
handone.add_finger(finger_ring);
handone.add_finger(finger_little);
handone.add_finger(finger_thumb);


return
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





