% test script for joint functionalities within the fingers and hand


clear all
close all
clc

%% test viapoint in the finger object
run examples\create_finger.m

figure(1)
finger_r.plot_finger()
rst_model = finger_r.update_rst_model;
rst_model.show(finger_r.q,'Frames','on')



%% set the joint limit on/off while updating q

limits_q = finger_r.limits_q;

finger_r.set_onejoint_limits_q(1,[-0.5,0.5]);
finger_r.set_onejoint_limits_q(2,[-0.1,pi/2]);
finger_r.set_onejoint_limits_q(3,[-0.1,pi/2]);
finger_r.set_onejoint_limits_q(4,[-0.1,pi/2]);

q = 2*ones(finger_r.nj,1);
finger_r.set_joint_limits_off;
finger_r.update_finger(q);
par_finger = finger_r.plot_parameter_init;

figure(2)
finger_r.plot_finger(par_finger)

finger_r.set_joint_limits_on;
finger_r.update_finger
par_finger.linecolor = 'b';
finger_r.plot_finger(par_finger)




