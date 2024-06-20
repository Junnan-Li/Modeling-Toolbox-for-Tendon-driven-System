% test the IK algorithms of toolbox and RST toolbox



clear all
close all
clc


%% create fingers
% use mdh to create finger
mdh_parameter = rand(7,4);
mdh_parameter(:,3) = 0;
mdh_parameter(1,1:4) = 0;
mdh_struct = mdh_matrix_to_struct(mdh_parameter, 1);
finger_r = Finger('Index', 'mdh',mdh_struct );

mdh_default_struct = finger_r.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
finger_r.set_mdh_parameters(mdh_matrix);
%% set random states
% set random base position and orientation
finger_r.w_p_base = 4*rand(3,1);
finger_r.w_R_base = euler2R_XYZ(rand(1,3));
finger_r.update_rst_model;
% joint configurations

q_r = rand(finger_r.nj,1);

% udpate finger with given joint configurations
finger_r.update_finger(q_r);
% finger_r.update_finger([0;0;0;0]);


% load rst model from finger class
rst_model = finger_r.rst_model;

% plot model
plot_par = finger_r.plot_parameter_init;
plot_par.axis_len = 0.4;

figure(1)
finger_r.plot_finger(plot_par)
hold on
axis equal
show(rst_model,q_r,'Collisions','on','Visuals','off');
hold on

%%


q_init = rand(finger_r.nj,1);
q_des = rand(finger_r.nj,1);
finger_r.update_finger(q_des);
x_des = finger_r.get_x_ee_w();
T_all_des = finger_r.get_T_all_links();
T_des = T_all_des(:,:,end);
finger_r.update_finger(q_init);
finger_r.update_rst_model;
rst_model = finger_r.rst_model;

[q_res, ~, x_res,phi_x,iter] = finger_r.invkin_numeric(x_des,200,[1e-6,1e-4],0.01);

error_q = q_des - q_res;
error_x = x_des - x_res;
if max(abs(error_x(:))) > 1e-4 
    fprintf('Test IK (invkin_numeric.m): failed! \n')
else
    fprintf('Test IK (invkin_numeric.m): pass! \n')
end

%%

ik = inverseKinematics("RigidBodyTree",rst_model);
weights = [1 1 1 1 1 1];

[configSoln,solnInfo] = ik("endeffector",T_des,weights,q_init);
figure(3)
plot3(x_des(1),x_des(2),x_des(3),'*','MarkerSize',15)
hold on
show(rst_model,q_init,'Collisions','on','Visuals','off');
hold on
show(rst_model,configSoln,'Collisions','on','Visuals','off');
