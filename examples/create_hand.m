

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

rst_model = finger_1.rst_model;

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
show(rst_model,[q;0],'Collisions','on','Visuals','off');
hold on
for i = 1:finger_1.nl
    
    contact_pos = finger_1.list_links(i).contacts(1).base_p;
    plot3(contact_pos(1),contact_pos(2),contact_pos(3),'*','Color', 'r', 'MarkerSize',10)
end




