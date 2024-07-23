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

muscle = finger.add_Muscle('muscle');
muscle.add_viapoints(vp1)
muscle.add_viapoints(vp2)
muscle.add_viapoints(vp3)
muscle.add_viapoints(vp4)
muscle.add_viapoints(vp5)




obs1 = finger.add_Obstacle_cylinder('Cyl1',1,[0,0,0]',eye(3),0.1,0.2);
obs2 = finger.add_Obstacle_cylinder('Cyl1',2,[0.05,0,0]',euler2R_XYZ([0.1,0,0]),0.1,0.2);

finger.update_obstacles;
% finger.plot_obstacles(plot_par)

%%
muscle.add_Muscle_Obstacles(obs1);
muscle.add_Muscle_Obstacles(obs2);

% finger.cal_all_Muscle_Length

finger.plot_muscles(plot_par)

figure(2)
for i = 1:3
    q = rand(finger.nj,1);
    finger.update_finger(q)
    finger.plot_finger(plot_par)
    finger.plot_muscles(plot_par)
end
%% update muscle constrain list

muscle.init_list_constr
%%
q = [0.8;0.3];
finger.update_finger(q);
figure(12)
muscle.plot_muscles(plot_par);

[l_total, wrap_status,w_PS_p] = muscle.cal_Muscle_length_ObstacleSet_Cyl_Garner;

for i = 1:length(wrap_status)
    if wrap_status(i)==1
        plot3(w_PS_p{i}(1,:)',w_PS_p{i}(2,:),w_PS_p{i}(3,:), ...
            'r.','MarkerSize',plot_par.muscle_markersize);
    end
end

%% symbolic 

[l_QT,w_status,w_Q_p,w_T_p] = cal_obstacle_vp_cyl_Garner(w_T_obs, w_P_p, w_S_p, radius, wrap_direction)
