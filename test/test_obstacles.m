% test script for viapoint and muscle object functionalities within the hand


clear all
close all
clc

%% create fingers
% use mdh to create finger
finger_r = create_finger_random('finger_Obstacle_test', 6);
% finger_r.set_base(zeros(3,1),euler2R_XYZ(zeros(1,3)))
%% set random states
rst_model = finger_r.update_rst_model;
% load rst model from finger class
rst_model = finger_r.rst_model;

q_r = rand(finger_r.nj,1);
finger_r.update_finger(q_r);
% plot model
plot_par = finger_r.plot_parameter_init;
plot_par.axis_len = 0.5;
plot_par.markersize = 10;
figure(1)
finger_r.plot_finger(plot_par)
hold on
axis equal
show(rst_model,q_r,'Collisions','on','Visuals','off');
hold on

%% obstacles
% 
Obs1 = finger_r.add_Obstacle('obstacle_1', 2, [0,0,0]',euler2R_XYZ([0,0,0]));
Obs2 = finger_r.add_Obstacle('obstacle_2', 2, [0,0,0]',euler2R_XYZ([pi/2,0,0]));
Obs3 = finger_r.add_Obstacle('obstacle_3', 2, [0,0,0]',euler2R_XYZ([pi,0,0]));
Obs4 = finger_r.add_Obstacle('obstacle_4', 2, [0,0,0]',euler2R_XYZ([pi/2*3,0,0]));
% 
% finger_r.update_finger(q_r);
% finger_r.plot_obstacles(plot_par)

%% Cylinder obstacles

Obsc1 = finger_r.add_Obstacle_cylinder('obstacle_1', 3, [0,0,0]',euler2R_XYZ([0,0,0]));
Obsc2 = finger_r.add_Obstacle_cylinder('obstacle_2', 3, [0,0,0]',euler2R_XYZ([pi/2,0,0]));
Obsc3 = finger_r.add_Obstacle_cylinder('obstacle_3', 3, [0,0,0]',euler2R_XYZ([pi,0,0]));
Obsc4 = finger_r.add_Obstacle_cylinder('obstacle_4', 3, [0,0,0]',euler2R_XYZ([pi/2*3,0,0]));

finger_r.update_finger(q_r);
finger_r.plot_obstacles(plot_par)


