% test script for obstacle set algorithm



clear all
close all
clc


finger_dimension = [1,1];

finger = Finger('Index', 'type','R_RR', 'l_links',finger_dimension);
mdh_default_struct = finger.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
% mdh_matrix(2,1) = -pi/2;
% mdh_matrix(2,4) = 1;
finger.set_mdh_parameters(mdh_matrix);


q = [0,0];
finger.update_finger(q);

plot_par = finger.plot_parameter_init;

figure(1)
finger.plot_finger(plot_par);
axis equal
vp1 = finger.base.add_viapoint_link('base_VP_1',[-0.5,-0.2,0]);
vp2 = finger.list_links(1).add_viapoint_link('L1_VP_1',[0.2,-0.1,0]);
vp3 = finger.list_links(1).add_viapoint_link('L1_VP_2',[0.7,-0.1,0]);
vp4 = finger.list_links(2).add_viapoint_link('L2_VP_1',[0.2,-0.1,0]);
vp5 = finger.list_links(2).add_viapoint_link('L2_VP_2',[0.5,-0.05,0]);


finger.update_list_viapoints

finger.plot_viapoints(plot_par)

muscle1 = finger.add_Muscle('muscle1');
muscle1.add_viapoints(vp1)
muscle1.add_viapoints(vp2)
muscle1.add_viapoints(vp3)
muscle1.add_viapoints(vp4)
muscle1.add_viapoints(vp5)



obs1 = finger.add_Obstacle_cylinder('Cyl1',1,[0,0,0]',eye(3),0.1,0.2);
obs2 = finger.add_Obstacle_cylinder('Cyl1',2,[0.05,0.02,0]',euler2R_XYZ([0.2,0.2,0]),0.1,0.2);

% finger.plot_obstacles(plot_par)

%%
muscle1.add_Muscle_Obstacles(obs1);
muscle1.add_Muscle_Obstacles(obs2);
finger.update_muscles

%% update muscle constrain list

muscle1.init_list_constr

%%

plot_par = finger.plot_parameter_init;
q = [1.2;1.3];
finger.update_finger(q);
figure(12)
[l_total, wrap_status,w_PS_p] = muscle1.cal_Muscle_length_ObstacleSet_Cyl_Garner;

%% calculate moment arm
% difference
tic
q1 = [1.2;1.3];
finger.update_finger(q1);
[l_11_num, wrap_status,w_PS_p] = muscle1.cal_Muscle_length_ObstacleSet_Cyl_Garner;
% l_11_num = muscle1.cal_muscle_length;
finger.plot_finger(plot_par)
muscle1.plot_muscles(plot_par);
% muscle2.plot_muscles(plot_par);
q2 = [1.2;1.3] + [0;0.001];
finger.update_finger(q2);
[l_12_num, wrap_status,w_PS_p] = muscle1.cal_Muscle_length_ObstacleSet_Cyl_Garner;
% l_12 = muscle1.cal_muscle_length;
finger.plot_finger(plot_par)
muscle1.plot_muscles(plot_par);
% muscle2.plot_muscles(plot_par);

ma_numeric = (l_12_num-l_11_num)./(q2-q1)
t1 = toc;
% 
tic
finger.update_finger(q1);
[l_total, wrap_status,w_PS_p] = muscle1.cal_Muscle_length_ObstacleSet_Cyl_Garner;
f1 = w_PS_p{2}(:,1) - muscle1.constr_obs{2}{1}.w_p_VP;
f1_norm = f1/norm(f1);
f2 = muscle1.constr_obs{2}{3}.w_p_VP - w_PS_p{2}(:,2);
f2_norm = f2/norm(f2);
J2 = cross(muscle1.constr_obs{2}{2}.Link.w_T_Link(1:3,3),w_PS_p{2}(:,1)-muscle1.constr_obs{2}{2}.Link.w_T_Link(1:3,4));
J3 = cross(muscle1.constr_obs{2}{2}.Link.w_T_Link(1:3,3),w_PS_p{2}(:,2)-muscle1.constr_obs{2}{2}.Link.w_T_Link(1:3,4));
J4 = cross(muscle1.constr_obs{2}{2}.Link.w_T_Link(1:3,3),muscle1.constr_obs{2}{3}.w_p_VP - muscle1.constr_obs{2}{2}.Link.w_T_Link(1:3,4));
ma_ = J2'*f1_norm + J3'* (-f2_norm) + J4'*f2_norm
t2 = toc;
%% a muscle with obstacle whose insertion is on next link

muscle2 = finger.add_Muscle('muscle2');
muscle2.add_viapoints(vp1)
muscle2.add_viapoints(vp4)
% muscle2.add_viapoints(vp5)

muscle2.add_Muscle_Obstacles(obs1);
% finger.update_obstacles;
finger.update_muscles
muscle2.init_list_constr


tic
q1 = [1.2;1.3];
finger.update_finger(q1);
[l_11_num, wrap_status,w_PS_p] = muscle2.cal_Muscle_length_ObstacleSet_Cyl_Garner;
finger.plot_finger(plot_par)
muscle2.plot_muscles(plot_par);
q2 = [1.2;1.3] + [0;0.001];
finger.update_finger(q2);
[l_12_num, wrap_status,w_PS_p] = muscle2.cal_Muscle_length_ObstacleSet_Cyl_Garner;
finger.plot_finger(plot_par)
muscle2.plot_muscles(plot_par);
ma_numeric = (l_12_num-l_11_num)./(q2-q1)
t1 = toc;
% 
tic
finger.update_finger(q1);
[l_total, wrap_status,w_PS_p] = muscle2.cal_Muscle_length_ObstacleSet_Cyl_Garner;
f1 = w_PS_p{1}(:,1) - muscle2.constr_obs{1}{1}.w_p_VP;
f1_norm = f1/norm(f1);
f2 = muscle2.constr_obs{1}{3}.w_p_VP - w_PS_p{1}(:,2);
f2_norm = f2/norm(f2);
J2 = cross(muscle2.constr_obs{1}{2}.Link.w_T_Link(1:3,3),w_PS_p{1}(:,1)-muscle2.constr_obs{1}{2}.Link.w_T_Link(1:3,4));
J3 = cross(muscle2.constr_obs{1}{2}.Link.w_T_Link(1:3,3),w_PS_p{1}(:,2)-muscle2.constr_obs{1}{2}.Link.w_T_Link(1:3,4));
J4 = cross(muscle2.constr_obs{1}{2}.Link.w_T_Link(1:3,3),muscle2.constr_obs{1}{3}.w_p_VP - muscle2.constr_obs{1}{2}.Link.w_T_Link(1:3,4));
ma_ = J2'*f1_norm + J3'* (-f2_norm) + J4'*f2_norm
t2 = toc;

%% hand 
% test obstacle and via points in hand objects
hand = create_hand_random('hand_Obstacle_Set', [2,2,3,4] );
% hand_rst = hand.update_rst_model;
hand.add_finger(finger)
% hand.update_hand_list

par_plot = hand.plot_parameter_init();

hand.add_Muscle(muscle1)
hand.add_Muscle(muscle2)
q = rand(hand.nj,1);
hand.update_hand(q);

figure(4)
hand.plot_hand(par_plot)
hand.plot_hand_obstacles(par_plot)
hand.plot_hand_viapoints(par_plot)
hand.plot_hand_muscles(par_plot)
axis equal

[l_11_num, wrap_status,w_PS_p] = muscle1.cal_Muscle_length_ObstacleSet_Cyl_Garner;
[l_12_num, wrap_status2,w_PS_p2] = muscle2.cal_Muscle_length_ObstacleSet_Cyl_Garner;
%% validate the hand.cal_hand_Muscle_l_J_Garner in terms of Jacobian
% test moment arm calculation of 40 muscles
% add muscles
clc
for i = 1:40
    m3 = copy(muscle1);
    hand.add_Muscle(m3)
end

%%
q_init = rand(hand.nj,1);
hand.update_hand(q_init);
hand.plot_hand(par_plot)
hand.plot_hand_viapoints(par_plot)
hand.plot_hand_muscles(par_plot)
tic
[length1,J1, wrap_status1,w_PS_p1] = hand.cal_hand_Muscle_l_J_Garner(hand.get_w_T_links_inhand);
t1 = toc;
%
tic
J2 = hand.get_Muscle_Momentarm_1st_f; 
t2 = toc;

tic
J3 = hand.get_Muscle_Momentarm_1st_c;
t3 = toc;
J_error1 = J1 - J2;
J_error2 = J1 - J3;
J_error = [J_error1;J_error2];

if max(abs(J_error(:))) > 1e-6 
    fprintf('Test (muscle Jacobian): failed! \n')
else
    fprintf('Test (muscle Jacobian): pass! \n')
end
fprintf('------------------------------------- \n')
fprintf('cal_hand_Muscle_l_J_Garner: %f! \n',t1)
fprintf('get_Muscle_Momentarm_1st_f: %f! \n',t2)
fprintf('get_Muscle_Momentarm_1st_c: %f! \n',t3)

%% gradient-based optimization method 


w_T_obs = eye(4);
radius = 1;
w_P_p = [-4,-3,0];
w_S_p = [4,-3,0];
wrap_direction = 1;

[l_muscle,w_status,w_Q_p,w_T_p] = cal_obstacle_vp_cyl_Garner(w_T_obs, w_P_p, w_S_p, radius, wrap_direction );

figure(10)
plot3(w_P_p(1),w_P_p(2),w_P_p(3),'r.')
hold on
plot3(w_S_p(1),w_S_p(2),w_S_p(3),'r.')
hold on
[X,Y,Z] = cylinder_vertics_to_plot(w_T_obs, radius,1,50);
s = surf(X,Y,Z,'FaceColor','b', 'EdgeColor','none');
alpha(s,0.2)
hold on
axis equal
plot3(w_Q_p(1),w_Q_p(2),w_Q_p(3),'r.')
plot3(w_T_p(1),w_T_p(2),w_T_p(3),'r.')



