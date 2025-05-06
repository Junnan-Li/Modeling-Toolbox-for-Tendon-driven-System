% create a finger object with random mdh, base configuration, and dynamic parameters
% 
% input:
%           finger_name:        name of the finger object
%           finger_dof:         number of joints 
% 
% output:   
%           finger_obj:         Finger object
% 
function finger_obj = create_finger_random_par_T(finger_name, finger_dof)


% create zeros mdh parameters
mdh_parameter = zeros(finger_dof+1,4);
mdh_parameter(:,3) = 0;
mdh_struct = mdh_matrix_to_struct(mdh_parameter, 1);
finger_obj = Finger(finger_name, 'mdh',mdh_struct );
% changing Finger.par_T_link randomly
for i = 1:finger_obj.nl+1 
    finger_obj.set_par_T_link(i, [eul2rotm(rand(1,3),'XYZ'),rand(3,1);0 0 0 1]);
end
%% set random base
finger_obj.set_base(rand(3,1),euler2R_XYZ(rand(1,3)))

% joint configurations
q_r = rand(finger_dof,1);
finger_obj.update_finger(q_r);

%% set random dynamic parameters
finger_obj.set_base_dynpar(rand(1),rand(3,1),[rand(3,1);zeros(3,1)] );
for i = 1:finger_obj.nl
    finger_obj.list_links(i).set_mass(rand(1));
    finger_obj.list_links(i).set_com(rand(3,1));
    finger_obj.list_links(i).set_inertia([rand(3,1);zeros(3,1)]);
end
finger_obj.update_finger_par_dyn;
finger_obj.update_rst_model;
end