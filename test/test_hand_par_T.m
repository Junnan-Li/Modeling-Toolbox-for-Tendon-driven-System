% test Hand and associated dynamics


close all
clear all
clc


%% create hand with two fingers

base_1 = create_finger_random('base_1', 4);
for i = 1:base_1.nl
    base_1.set_par_T_link(i, [eul2rotm(rand(1,3),'XYZ'),rand(3,1);0 0 0 1]);
end

base_2 = create_finger_random('base_2', 4);
for i = 1:base_2.nl
    base_2.set_par_T_link(i, [eul2rotm(rand(1,3),'XYZ'),rand(3,1);0 0 0 1]);
end

finger_1 = create_finger_random('finger_1', 4);
for i = 1:finger_1.nl
    finger_1.set_par_T_link(i, [eul2rotm(rand(1,3),'XYZ'),rand(3,1);0 0 0 1]);
end
finger_2 = create_finger_random('finger_2', 4);
for i = 1:finger_2.nl
    finger_2.set_par_T_link(i, [eul2rotm(rand(1,3),'XYZ'),rand(3,1);0 0 0 1]);
end
finger_3 = create_finger_random('finger_3', 4);
for i = 1:finger_3.nl
    finger_3.set_par_T_link(i, [eul2rotm(rand(1,3),'XYZ'),rand(3,1);0 0 0 1]);
end

% 
hand = Hand('hand_exp');
hand.add_base(base_1);
hand.add_base(base_2);
hand.add_finger(finger_1);
hand.add_finger(finger_2);
hand.add_finger(finger_3);
q_hand = rand(hand.nj,1);
hand.update_hand(q_hand);
hand.update_hand_par_dyn;

par_plot_hand = hand.plot_parameter_init();
par_plot_hand.axis_len = 0.4;
hand_rst = hand.update_rst_model;
hand_rst.show(q_hand,'Frames','on');
hold on
axis equal
hand.plot_hand(par_plot_hand)
hand.plot_hand_com(par_plot_hand);

%% check if end effector posistion is correct
test_failed = 0;
for i = 1:20
    q_hand = rand(hand.nj,1);
    hand.update_hand(q_hand);

    w_T_ee_all = hand.get_w_T_ee_all;
    T_1_rst = hand_rst.getTransform(q_hand, 'finger_1_endeffector');
    T_2_rst = hand_rst.getTransform(q_hand, 'finger_2_endeffector');
    T_3_rst = hand_rst.getTransform(q_hand, 'finger_3_endeffector');
    T_rst = [T_1_rst;T_2_rst;T_3_rst];

    T_error = w_T_ee_all-T_rst;
    if max(abs(T_error(:))) > 1e-10
        test_failed = 1;
    end
end
if test_failed == 1
    fprintf('Test Hand position: failed! \n')
else
    fprintf('Test Hand position: pass! \n')
end
%% test the Jacobian function 
% Jacobian_geom_w_one_finger
test_failed = 0;
for i = 1:20
    q_hand = rand(hand.nj,1);
    hand.update_hand(q_hand);
    hand_rst = hand.update_rst_model;
    jacobian_f1 = geometricJacobian(hand_rst,q_hand,'finger_1_endeffector');
    jacobian_f1_mod = [jacobian_f1(4:6,:);jacobian_f1(1:3,:)];
    
    jacobian_f2 = geometricJacobian(hand_rst,q_hand,'finger_2_endeffector');
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
    fprintf('Test Hand Jacobian: failed! \n')
else
    fprintf('Test Hand Jacobian: pass! \n')
end

%%  Jacobian 
% Jacobian_geom_w_point

test_failed = 0;
for i = 1:20
    q = rand(hand.nj,1);
    hand.update_hand(q);
    plot_par = hand.plot_parameter_init;
%     figure(10)
%     hand.plot_hand(plot_par)
%     axis equal
%     hand_rst.show(q,'Frames','on');
%     w_T_all = hand.get_w_T_links_inhand;
%     plot3(reshape(w_T_all(1,4,:),hand.nl,1),reshape(w_T_all(2,4,:),...
%         hand.nl,1),reshape(w_T_all(3,4,:),hand.nl,1),'k+','MarkerSize',15)
    J_rst = nan(6,hand.nl,hand.nl);
    J = nan(6,hand.nl,hand.nl);
    J1 = nan(6,hand.nl,hand.nl);
    for ii = 1:hand.nl
        link_i = hand.list_links(ii);
        J_rst_tmp = geometricJacobian(hand_rst,q,link_i.name);
        J_rst(:,:,ii) = [J_rst_tmp(4:6,:);J_rst_tmp(1:3,:)];
        [J_trans,J_all] = hand.Jacobian_geom_w_point(ii,[0,0,0]);
        J(:,:,ii) = J_all;
    end
    J_error = J_rst - J;
    if max(abs(J_error(:))) > 1e-10
        test_failed = 1;
    end
end
if test_failed == 1
    fprintf('Test Hand Jacobian: failed! \n')
else
    fprintf('Test Hand Jacobian: pass! \n')
end


%% 
hand.update_sim_par;
test_failed = 0;

for test_iter = 1:20
%     hand

    q = rand(hand.nj,1);
    qd = rand(hand.nj,1);
    qdd = rand(hand.nj,1);

    hand.update_hand(q);

    tau_rst = hand_rst.inverseDynamics(q,qd,qdd); % gravity torque^^
    Tau = hand.invdyn_ne_hand_w_end(q,qd,qdd);

    error_tau = tau_rst-Tau;
    if max(abs(error_tau(:))) > 1e-10
        test_failed = 1;
        break
    end
end
if test_failed == 1
    fprintf('Test (ID): failed! \n')
else
    fprintf('Test (ID): pass! \n')
end







