% generate the finger model from the urdf file specifically for hand
% prototype (modified the mdh parameter from urdf model due to the -Z rotation axis)
%
% 06.2024 Ljn

function  Finger_model = create_handone_finger_from_urdf(urdf_file,finger_name)


Finger_model = init_from_rst(urdf_file,finger_name);
mdh_ori = Finger_model.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_ori,1);
mdh_matrix(5,2) = 0.014;
mdh_matrix(1,1) = pi;
% mdh_matrix(1,2) = 0.03;
Finger_model.set_mdh_parameters(mdh_matrix);
Finger_model.set_base([0;0;0],euler2R_XYZ([0,0,0]));
Finger_model.update_finger(zeros(Finger_model.nj,1));
% update rst model
Finger_model.update_rst_model;

end


