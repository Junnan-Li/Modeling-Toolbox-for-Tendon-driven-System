% test the IK algorithms of Finger object and RST model



clear all
close all
clc


%% create fingers
% use mdh to create finger
finger_r = create_finger_random('finger_random', 6);
% % finger_r.set_base(zeros(3,1),euler2R_XYZ(zeros(1,3)))
%% set random states
finger_r.update_rst_model;
% load rst model from finger class
rst_model = finger_r.rst_model;

q_r = rand(finger_r.nj,1);
finger_r.update_finger(q_r);
% plot model
plot_par = finger_r.plot_parameter_init;
plot_par.axis_len = 0.4;

figure(1)
finger_r.plot_finger(plot_par)
hold on
axis equal
show(rst_model,q_r,'Collisions','on','Visuals','off');
hold on

%% define ik problem

q_init = rand(finger_r.nj,1); % init finger pose
q_des = rand(finger_r.nj,1); % desired finger pose
finger_r.update_finger(q_des);
x_des = finger_r.get_x_ee_w(); % fingertip position of desired pose
T_all_des = finger_r.get_T_all_links();
T_des = T_all_des(:,:,end);
finger_r.update_finger(q_init); 
finger_r.update_rst_model;
rst_model = finger_r.rst_model;

IK_par = IK_par();
% IK_par.ikpar_NR.visual = 1;
IK_par.ikpar_NR.alpha = 0.3;
tic
[q_ik_NR, info_NR] = finger_r.invkin_numeric(x_des,IK_par);
t1 = toc;
finger_r.update_finger(q_init); 
tic;
[q_ik_LM, info_LM] = finger_r.invkin_numeric_LM(x_des);
t2 = toc;
tic
[q_ik_rst, solnInfo] = finger_r.invkin_rst(x_des);
t3 = toc;
error_q = q_des - q_ik_NR;
error_q_LM = q_des - q_ik_LM;
error_x = x_des - info_NR.x_res;
error_x_LM = x_des - info_LM.x_res;

% 
figure(3)
plot3(x_des(1),x_des(2),x_des(3),'*','MarkerSize',15);
hold on
show(rst_model,q_des,'Collisions','on','Visuals','off');
hold on
show(rst_model,q_ik_rst,'Collisions','on','Visuals','off');
% finger_r.plot_finger(plot_par)

fprintf('IK Test: \n')
fprintf('Finger IK Test (invkin_numeric NR):    status: %d; time: %f; iter: %d \n', info_NR.status,t1,info_NR.iter)
fprintf('Finger IK Test (invkin_numeric_LM LM): status: %d; time: %f; iter: %d \n', info_LM.status,t2,info_LM.iter)
fprintf('Finger IK Test (invkin_rst):           status: %d; time: %f; iter: %d \n', solnInfo.ExitFlag==1,t3,solnInfo.Iterations)
fprintf('############## \n')
%% create hand

hand = create_hand_random('hand_IK_test', [2,2,2,4] );

hand_rst = hand.update_rst_model;
q = rand(hand.nj,1);
hand.update_hand(q);
figure(4)
hand.plot_hand()
hand_rst.show(q,'Frames','on');

%% test jacobian

finger_1_ee_name = 'finger_1_endeffector';
finger_2_ee_name = 'finger_2_endeffector';
test_failed = 0;
for i = 1:20
    q_hand = rand(hand.nj,1);
    hand.update_hand(q_hand);
    hand_rst = hand.update_rst_model;
    jacobian_f1 = geometricJacobian(hand_rst,q_hand,finger_1_ee_name);
    jacobian_f1_mod = [jacobian_f1(4:6,:);jacobian_f1(1:3,:)];
    
    jacobian_f2 = geometricJacobian(hand_rst,q_hand,finger_2_ee_name);
    jacobian_f2_mod = [jacobian_f2(4:6,:);jacobian_f2(1:3,:)];

    J = hand.Jacobian_geom_w_one_finger(1,q_hand);
    J2 = hand.Jacobian_geom_w_one_finger(2,q_hand);
    J_error = jacobian_f1_mod-J;
    J_error2 = jacobian_f2_mod-J2;

    if max(abs(J_error(:))) > 1e-10 | max(abs(J_error2(:))) > 1e-10
        test_failed = 1;
    end
end
if test_failed == 1
    fprintf('Hand Test (Jacobian): failed! \n')
else
    fprintf('Hand Test (Jacobian): pass! \n')
end

%% IK of hand

% singer fingertip 
par_plot_hand = hand.plot_parameter_init();
par_plot_hand.axis_len = 0.1;
par_plot_hand.linecolor = 'b';


q_hand = rand(hand.nj,1);
hand.update_hand(q_hand);
w_T_ee_all = hand.get_w_T_ee_all;
x_des_1 = [w_T_ee_all(1:3,4);R2euler_XYZ(w_T_ee_all(1:3,1:3))];
hand.plot_hand(par_plot_hand);

q_hand_init = rand(hand.nj,1);
hand.update_hand(q_hand_init);

par_plot_hand.linecolor = 'r';


% ik using rst toolbox
tic
[q_res_rst, solnInfo] = hand.invkin_rst(x_des_1,1);
t1 = toc;
% 
hand.update_hand(q_hand_init);
tic
[q_res1, info2] = hand.invkin_numeric(x_des_1,1);
t2 = toc;
% hand.plot_hand(par_plot_hand);

% LM method
hand.update_hand(q_hand_init);
tic 
[q_res2, info3] = hand.invkin_numeric_LM(x_des_1,1);
t3 = toc;

fprintf('Hand IK Test (invkin_numeric NR):    status: %d; time: %f; iter: %d \n', info2.status,t2,info2.iter)
fprintf('Hand IK Test (invkin_numeric_LM LM): status: %d; time: %f; iter: %d \n', info3.status,t3,info3.iter)
fprintf('Hand IK Test (invkin_rst):           status: %d; time: %f; iter: %d \n', solnInfo.ExitFlag==1,t1,solnInfo.Iterations)

