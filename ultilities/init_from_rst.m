




function finger_init = init_from_rst(rst_model)
% build/update the rst model of the finger based on the mdh
% parameters
%
% TODO: integrate the dynamic parameters of the links into
% model


%% rigidbodytree kinematic model to obj

w_p_base = 4*zeros(3,1);
w_R_base = euler2R_XYZ(zeros(1,3));
nj = length(rst_model.Bodies);
mdh_finger = zeros(nj+1,4);
mdh_finger(1,:) = [0,0,0,0]; % base


for i = 1:nj
    T_i = rst_model.Bodies{i}.Joint.JointToParentTransform;
    phi_i = R2euler_XYZ(T_i(1:3,1:3));
    p_i = T_i(1:3,4);
    mdh_finger(i,:) = [phi_i(1),p_i(1),0,p_i(3)];


end
mdh_struct = mdh_matrix_to_struct(mdh_finger, 1);
finger_init = Finger('finger_rst', 'mdh',mdh_struct );
finger_init.w_p_base = w_p_base;
finger_init.w_R_base = w_R_base;
finger_init.update_finger(finger_init.q_a);
%% rigidbodytree dynamic parameters to obj

for i = 1:nj
    mass_i = rst_model.Bodies{i}.Mass;
    finger_init.list_links(i).set_mass(mass_i);
    com_i = rst_model.Bodies{i}.CenterOfMass';
    finger_init.list_links(i).set_com(com_i);
    inertia_tmp = rst_model.Bodies{i}.Inertia';
    inertia_i = inertia_tmp - mass_i * [com_i(2)^2+com_i(3)^2;com_i(1)^2+com_i(3)^2;com_i(1)^2+com_i(2)^2;...
                    -com_i(2)*com_i(3);-com_i(1)*com_i(3);-com_i(1)*com_i(2)];
    finger_init.list_links(i).set_inertia(inertia_i);

end
finger_init.update_finger_par_dyn;


end