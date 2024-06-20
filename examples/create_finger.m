%   example script for creating an example of finger mdh

%% create finger with tendons

clear all 
close all
clc

%% define a finger with conventianal configuration
% four joints
finger_dimension = [1,5,3,2]; % in meter
finger_r = Finger('Index', 'type','R_RRRR', 'l_links',finger_dimension); 
% finger_r = Finger('hand_one','mdh',)
mdh_default_struct = finger_r.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
% mdh_matrix(2,4) = 1;
finger_r.set_mdh_parameters(mdh_matrix);

%%

% mdh_struct = mdh_matrix_to_struct(mdh_parameter, 1);
% finger_r = Finger('Index', 'mdh',mdh_struct );


%% basic settings
% set base position and orientation
finger_r.set_base(4*rand(3,1),euler2R_XYZ(zeros(1,3)));

% init joint configurations
q_0 = [pi/4;pi/4;1;1];

% udpate finger with given joint configurations
finger_r.update_finger(q_0);
finger_r.update_rst_model;

% set dynamic parameters

finger_r.list_links(1).set_mass(0.04); % in kg
finger_r.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(2).set_mass(0.03); % in kg
finger_r.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(3).set_mass(0.02); % in kg
finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(4).set_mass(0.02); % in kg
finger_r.list_links(4).set_inertia([0.5,0.2,0.2,0,0,0]); 
% update dynamic parameters
finger_r.update_finger_par_dyn;
finger_r.update_rst_model;
% finger_r.update_finger(q_0);


return
%% add tendons

if finger_r.nt == 0
    finger_r.add_tendon('Flex_1', [1,1,1,1]);
    finger_r.add_tendon('Flex_2', [1,1,1,0]);
    finger_r.add_tendon('Flex_3', [1,1,0,0]);
    finger_r.add_tendon('Flex_4', [1,0,0,0]);
    finger_r.add_tendon('Ext_1', [-1,-1,-1,-1]);
    finger_r.add_tendon('Ext_2', [-1,-1,-1,0]);
    finger_r.add_tendon('Ext_3', [-1,-1,0,0]);
    finger_r.add_tendon('Ext_4', [-1,0,0,0]);
end

% finger_r.set_tendon_par_MA_poly3(1,1,[0,0,0.01,0.03]);
% finger_r.set_tendon_par_MA_poly3(3,1,[0,0,0.01,0.3]);
% finger_r.set_tendon_par_MA_poly3(3,2,[0,0,0.01,0.035]);
% finger_r.set_tendon_par_MA_poly3(4,1,[0,0,0.001,0.02]);
finger_r.update_finger(q_0);
finger_r.update_M_coupling(q_0);
finger_r.M_coupling;

% set joint limits
finger_r.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints

finger_r.update_joints_info;

%% add contact
if finger_r.nc == 0
    for i = 1:finger_r.nl-1
        b_R_i = finger_r.list_links(i).base_R;
        b_p_i1 = finger_r.list_links(i+1).base_p;
        b_p_i = finger_r.list_links(i).base_p;
        finger_r.list_links(i).add_contact((b_R_i'*(b_p_i1-b_p_i))/2);
%         finger_r.list_links(i).add_contact([finger_r.list_links(i).Length 0 0]');
    end
    mdh_ori = mdh_struct_to_matrix(finger_r.mdh_ori(end,:), 1);
    T_end = T_mdh_multi(mdh_ori(end,:));
    finger_r.list_links(finger_r.nl).add_contact(T_end(1:3,4)/2);
end
finger_r.update_list_contacts; % update link


return
%% plot the link origin position and all contacts
figure(1)
% plot 2 fingers
p_link_all_w_r = finger_r.get_p_all_links;
finger_r.rst_model.show(q_0)

hold on
plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','r');

hold on
w_p_contacts_all = finger_r.get_p_all_contacts;
plot3(w_p_contacts_all(1,:)',w_p_contacts_all(2,:)',w_p_contacts_all(3,:)','*','Color','b');

axis equal
return
%%

clear all
close all
clc


%% create fingers
finger_1 = Finger('Index', 'RRRR', [0.5 0.3 0.2]);
finger_2 = Finger('Middle', 'RRRR', [0.6 0.4 0.2]);
finger_3 = Finger('Ring', 'RRRR', [0.5 0.3 0.2]);
finger_4 = Finger('little', 'RRRR', [0.4 0.25 0.2]);

finger_list = {finger_1,finger_2,finger_3,finger_4};

%% set finger init position & orientation
finger_1.w_p_base = [1,0,0]';
finger_1.w_R_base = eul2rotm([pi/6,0,0],'ZYZ');

% set finger joint configurations
q = [45 0 0 0]'*pi/180;

% update
finger_1.update_finger(q);
% finger_1.update_rst_model;

rst_model_r = finger_1.rst_model;

% rst_model.show
% showdetails(rst_model)



mdh = mdh_struct_to_matrix(finger_1.mdh,1);
T = T_mdh_multi(mdh);


% geometric Jacobian of the endeffector
J_e = finger_1.Jacobian_geom_end;

%% add contact

for i = 1:finger_1.nl
    finger_1.list_links(i).add_contact([finger_1.list_links(i).Length/2 0 0]');
%     finger_1.list_links(i).add_contact([finger_1.list_links(i).Length/2 0 0]')
end

finger_1.update_all_contacts;


%% geometric Jacobian of the contact points

J_c = [];

for i = 1:finger_1.nl
    if finger_1.list_links(i).nc
        for j = 1:finger_1.list_links(i).nc
            J_c = [J_c;finger_1.Jacobian_geom_contact(q,finger_1.list_links(i).contacts(j))];
        end
    end
end



%% visualization
show(rst_model_r,[q;0],'Collisions','on','Visuals','off');
hold on
for i = 1:finger_1.nl
    
    contact_pos = finger_1.list_links(i).contacts(1).base_p;
    plot3(contact_pos(1),contact_pos(2),contact_pos(3),'*','Color', 'r', 'MarkerSize',10)
end

return







