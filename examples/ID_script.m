% For inverse dynamic calculation for Christ scenarios
% input:
%           q,qd,qdd:           [4x1]
%           w_b_x_obj:          [6x1] position and orientation of object
%                                     to the base represented in world frame
%           w_F_obj:            [6x1] object wrench of com represented in
%                                     world frame
% 
% 
% output:   
%           tau:                [4x1] joint torque from ID
%

clear all 
close all
clc
%% load model

run create_Dexmart_index.m
close all
model = Dexmart_index;

%% input

q_in = zeros(4,1); % random pos
qd_in = rand(4,1); % random pos
qdd_in = rand(4,1); % random pos
w_b_x_obj = [0.2,0,0,0,0,0]'; % random onj position

w_F_obj = rand(6,1); % random wrench -- external force

model.update_finger(q_in);
model.plot_finger
model.plot_com
axis equal

plot_par = model.plot_parameter_init;
plot_par.markercolor = 'k';
plot_par.markersize = 20;
%% 

w_b_p_obj = w_b_x_obj(1:3);

W_T_b = model.get_W_T_B;
w_p_obj = w_b_p_obj + W_T_b(1:3,4);

[w_p_ee,w_R_ee] = model.get_T_ee_w;
w_ee_p_obj = w_p_obj-w_p_ee;
ee_p_obj = w_R_ee' * w_ee_p_obj;
 

Contact_1 = model.add_contact('object', 4, ee_p_obj); % take object as contact
Contact_ee = model.add_contact('object', 4, [0.0285;0;0]); % define a contact of ee for testing

J_ee = model.Jacobian_geom_b_end(q_in);
J_contact_ee = model.Jacobian_geom_b_contact(q_in,Contact_ee);
error = J_ee-J_contact_ee;


[w_p_contacts_all,b_p_contacts_all] = model.get_p_all_contacts;


model.plot_contacts(plot_par);

model.update_finger(rand(4,1))
model.plot_finger
model.plot_com
model.plot_contacts(plot_par);

% calculate ID
Tau_link = model.invdyn_ne_w_end(q_in,qd_in,qdd_in, zeros(6,1));
Tau_obj = J_contact_ee'*w_F_obj;

Tau_gesamt = Tau_link + Tau_obj
