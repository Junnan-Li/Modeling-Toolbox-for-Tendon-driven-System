% finger class function test
% 


clear all
close all
clc


symbolic_test = 0;

%% create random finger
finger_T = create_finger_random('finger_example', 4);
plot_par = finger_T.plot_parameter_init();
plot_par.axis_len = 0.5;
q = rand(finger_T.nj,1);
finger_T.update_finger(q)
finger_T.plot_finger(plot_par);
for i = 1:finger_T.nl
    finger_T.set_par_T_link(i, [eul2rotm(rand(1,3),'XYZ'),rand(3,1);0 0 0 1]);
end
q = rand(finger_T.nj,1);
finger_T.update_finger(q)
rst_model = finger_T.update_rst_model;
plot_par = finger_T.plot_parameter_init();
plot_par.axis_len = 0.5;
finger_T.plot_finger(plot_par);
show(rst_model,q,'Frames','on');
axis equal
grid on

fprintf('Test Finger.kin_use_T: %d \n', finger_T.kin_use_T)

%% Test 1:  Transformation matrix test with respect to mdh parameters 
% 
q_r = rand(finger_T.nj,1);
finger_T.update_finger(q_r);

% transformnation matrix of each frame test
w_T_all = finger_T.get_T_all_links;
w_T_all_rst = zeros(4,4,finger_T.nj+2);
w_T_all_rst(:,:,1) = rst_model.Bodies{1}.Joint.JointToParentTransform;
for i = 1:finger_T.nj+1
    w_T_all_rst(:,:,i+1) = getTransform(rst_model,q_r,rst_model.BodyNames{i+1});
end 

% validaiton
T_error_all = max(abs(w_T_all_rst-w_T_all));
if max(T_error_all(:))> 1e-10
    fprintf('Test 1 (Transformation matrix): failed! \n')
else
    fprintf('Test 1 (Transformation matrix): pass! \n')
end

%% Test 2: Jacobian test
% test the geometric Jacobian matrix of the endeffector
% Jacobian_geom_b_end.m
% Jacobian_geom_w_end.m

% set variable randomly
finger_T.set_base(rand(3,1), euler2R_XYZ(rand(3,1)));
finger_T.update_rst_model;
% joint configurations
q_r = rand(finger_T.nj,1);
finger_T.update_finger(q_r);

% load rst model from finger class
rst_model = finger_T.rst_model;

% number of active joints
num_nja = finger_T.nja;

% geometric Jacobian computed by the class function 'Jacobian_geom_b_end'
J_class_b = finger_T.Jacobian_geom_b_end(q_r);% with respect to the base frame
W_R_b = finger_T.w_R_base();
J_class = blkdiag(W_R_b,W_R_b)*J_class_b;

% geometric Jacobian computed by the class function 'Jacobian_geom_b_end'
J_class_w = finger_T.Jacobian_geom_w_end(q_r);% with respect to the base frame

% geometrical Jacobian using Jacobian_geom_mdh.m function
J_geom_func_b = Jacobian_geom_mdh(mdh_struct_to_matrix(finger_T.mdh_ori,1),q_r);
J_geom_func = blkdiag(W_R_b,W_R_b)*J_geom_func_b;

% geometric Jacobian computed by the rst toolbox
J_rst = geometricJacobian(rst_model,q_r,rst_model.BodyNames{end}); % J_rst = [omega_x omega_y omega_z v_x v_y v_z]' 
J_rst = [J_rst(4:6,1:num_nja);J_rst(1:3,1:num_nja)]; % J_rst = [v_x v_y v_z omega_x omega_y omega_z]'

% validaiton
J_error = abs(J_class-J_class_w);
J_error1 = abs(J_class-J_rst);
J_error2 = abs(J_geom_func-J_rst);

if max([J_error(:),J_error1(:),J_error2(:)]) > 1e-10
    fprintf('Test 2 (geometric Jacobian): failed! \n')
else
    fprintf('Test 2 (geometric Jacobian): pass! \n')
end

% analytical Jacobian
J_analytic_end_b = finger_T.Jacobian_analytic_b_end(q_r);
J_analytic_end = blkdiag(W_R_b,W_R_b)*J_analytic_end_b;

J_analytic_func_b = Jacobian_analytic_mdh(mdh_struct_to_matrix(finger_T.mdh_ori,1),q_r);
J_analytic_func = blkdiag(W_R_b,W_R_b)*J_analytic_func_b;



%% Test 3: Frame position test
% test properties
% Finger.

for t3 = 1:20
    test_3_error = 0;
    q_r = rand(finger_T.nj,1);
    finger_T.update_finger(q_r);

    % Base to World frame transformation of class model
    W_p_b = finger_T.w_p_base();
    W_R_b = finger_T.w_R_base();

    
    w_T_all = finger_T.get_T_all_links;
    T_rst_b = getTransform(finger_T.rst_model,q_r,finger_T.rst_model.BodyNames{1});
    T_rst_ee = getTransform(finger_T.rst_model,q_r,finger_T.rst_model.BodyNames{end});


    for i = 1:finger_T.nl
        % rsi model: frame position & orientation in world frame
        T_rst_link_i = getTransform(finger_T.rst_model,q_r,finger_T.list_links(i).name);
        w_T_link_i = finger_T.list_links(i).w_T_Link;

        b_T_link_i = pR2T(finger_T.list_links(i).base_p,finger_T.list_links(i).base_R);
        w_T_link_i_2 = finger_T.w_T_base * b_T_link_i;

        error_T1 = T_rst_link_i - w_T_link_i;
        error_T2 = T_rst_link_i - w_T_link_i_2;
        error_T3 = T_rst_link_i - w_T_all(:,:,i+1); 
        if max(abs([error_T1(:),error_T2(:),error_T3(:)])) > 1e-10
            test_3_error = 1;
        end
    end
end
% validaiton
if test_3_error == 1
    fprintf('Test 3 (Frame position): failed! \n')
else
    fprintf('Test 3 (Frame position): pass! \n')
end
%% Test 5_1 inverse dynamic test (fixed base)
% method 1: 
%       class function (fixed base): Finger.invdyn_ne_w_end(q_r,q_rD,q_rDD,F_ext)
% method 2: 
%       rst toolbox(fixed base): inverseDynamics(rst_model, q_r,q_rD,q_rDD, F_ext_rst);
% method 3: 
%       class function (floating base):  Finger.invdyn_ne_xq_fb_all_fext
% method 4:
%       class function (floating base, wt f_ext):  Finger.invdyn_ne_xq_fb_wt_fext_sub

% random states
q_r = rand(finger_T.nj,1);
finger_T.update_finger(q_r);
q_rD = rand(size(q_r));
q_rDD = rand(size(q_r));
F_ext = rand(6,1);

finger_T.set_base_dynpar(rand(1),rand(3,1),[rand(3,1);zeros(3,1)] );
for i = 1:finger_T.nl
    finger_T.list_links(i).set_mass(rand(1));
    finger_T.list_links(i).set_com(rand(3,1));
    finger_T.list_links(i).set_inertia([rand(3,1);zeros(3,1)]);
end
finger_T.update_finger_par_dyn;

Tau_class = finger_T.invdyn_ne_w_end(q_r,q_rD,q_rDD,F_ext);

% % floating base method
% X_base = [finger_T.w_p_base;R2euler_XYZ(finger_T.w_R_base)];
% XD_base = zeros(6,1);
% XDD_base = zeros(6,1);
% Tau_class_fb = finger_T.invdyn_ne_xq_fb_all_fext([X_base;q_r],[XD_base;q_rD],[XDD_base;q_rDD],[zeros(6,finger_T.nj+1),F_ext]);

% transfer the external force exerting on the 
rst_model = finger_T.update_rst_model;

transform = getTransform(rst_model,q_r,rst_model.BodyNames{end});
W_R_end = transform(1:3,1:3);
F_ext_rst = externalForce(rst_model,rst_model.BodyNames{end},[W_R_end'*F_ext(4:6);W_R_end'*F_ext(1:3)],q_r);
Tau_rst = inverseDynamics(rst_model, q_r,q_rD,q_rDD, F_ext_rst);

Tau_error = abs(Tau_class-Tau_rst);


if max(Tau_error(:)) > 1e-10
    fprintf('Test 5 (inverse dynamic): failed! \n')
else
    fprintf('Test 5 (inverse dynamic): pass! \n')
end
% return
%% Test 6 Forward dynamic test
% test Finger.fordyn_ne_w_end function with random F_ee 
test_failed = 0;
for i = 1:100
    % random states
    q_rD = rand(size(q_r));
    tau = rand(size(q_r));
    F_ext = [zeros(6,finger_T.nj+1),rand(6,1)];

    [qDD_class,M_fd,C_fd,G_fd] = finger_T.fordyn_ne_w_end(q_r,q_rD,tau,F_ext(:,end),0);
    
    % Lagrangian method 2
    w_T_all = finger_T.get_T_all_links;
    w_T_link = w_T_all(:,:,2:end-1);
    M_lag2 = invdyn_lag_T_M(w_T_link, finger_T.par_dyn_f.mass_all,...
        finger_T.par_dyn_f.com_all, finger_T.par_dyn_f.inertia_all);
    
    % transfer the external force exerting on the
    transform = getTransform(rst_model,q_r,rst_model.BodyNames{end});
    W_R_end = transform(1:3,1:3);
    F_ext_rst = externalForce(rst_model,rst_model.BodyNames{end},[W_R_end'*F_ext(4:6,end);W_R_end'*F_ext(1:3,end)],q_r);
    qDD_rst = forwardDynamics(rst_model,q_r,q_rD,tau,F_ext_rst);
    M_rst = rst_model.massMatrix(q_r);

    qDD_error = abs(qDD_class-qDD_rst);
    M_error = M_fd - M_rst;
    M_error_2 = M_rst - M_lag2;

    if max(abs([qDD_error(:);M_error(:);M_error_2(:)])) > 1e-10
        test_failed = 1;
    end
end
if test_failed
    fprintf('Test 6 (forward dynamic): failed! \n')
else
    fprintf('Test 6 (forward dynamic): pass! \n')
end

return

%% Test 8: inverse kinematic
% move to test_IK

p_link_all_w_r = finger_T.get_p_all_links;
figure(1)
plot3(finger_T.w_p_base(1),finger_T.w_p_base(2),finger_T.w_p_base(3),'x','MarkerSize',15);
hold on
plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','r');
hold on
grid on
axis equal


x_init = p_link_all_w_r(:,end);
x_des = x_init - 0.5*rand(3,1);
plot3(x_des(1),x_des(2),x_des(3),'*','Color','b','MarkerSize',20);
hold on


iter_max = 100;
alpha = 0.9;
color_plot = [1,0,0];
tol = 1e-9;
[q,q_all,x_res,phi_x,iter] = finger_T.invkin_trans_numeric(x_des,iter_max,tol,alpha);

p_link_all_w_r = finger_T.get_p_all_links;
plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','c');





