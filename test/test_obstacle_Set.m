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
t1 = toc
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
t2 = toc
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
t1 = toc
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
t2 = toc

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
%%
clc
[length1,J1, wrap_status1,w_PS_p1] = hand.cal_hand_Muscle_l_J_Garner;






