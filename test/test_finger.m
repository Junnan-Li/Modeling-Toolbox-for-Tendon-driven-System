% finger class function test
% 
% Test 1:
%       Transformation matrix test with given length and base
%       position & orientation
% Test 2: 
%       geometric Jacobian test of the end effector
% Test 3:
%       
% 
%           
% 


clear all
close all
clc


% define different types of fingers

% finger_list = {finger_r,finger_s,finger_p};

% finger_r = Finger('Index', 'type','R_RRRR', 'l_links',rand(1,4));

% use mdh to create finger
mdh_parameter = rand(5,4);
mdh_parameter(:,3) = 0;
mdh_parameter(1,1:4) = 0;
mdh_struct = mdh_matrix_to_struct(mdh_parameter, 1);
finger_r = Finger('Index', 'mdh',mdh_struct );

mdh_default_struct = finger_r.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix(2,1) = -pi/2;
finger_r.set_mdh_parameters(mdh_matrix);
%% set random states
% set random base position and orientation
finger_r.w_p_base = 4*rand(3,1);
finger_r.w_R_base = euler2R_XYZ(rand(1,3));
finger_r.update_rst_model;
% joint configurations

q_r = rand(4,1);

% udpate finger with given joint configurations
finger_r.update_finger(q_r);
% finger_r.update_finger([0;0;0;0]);


% load rst model from finger class
rst_model = finger_r.rst_model;

% plot rst model
show(rst_model,q_r,'Collisions','on','Visuals','off');

%% Test 1:  Transformation matrix test with respect to mdh parameters 
% mdh parameter from class properties
mdh = mdh_struct_to_matrix(finger_r.mdh,1);
b_T_class = T_mdh_multi(mdh);
W_T_b = finger_r.get_W_T_B();
T_class = W_T_b * b_T_class;

% mdh parameters from rst model
T_rst = getTransform(rst_model,q_r,'endeffector');

% validaiton
T_error = abs(T_class-T_rst);
if max(T_error(:)) > 1e-10
    fprintf('Test 1 (Transformation matrix): failed! \n')
else
    fprintf('Test 1 (Transformation matrix): pass! \n')
end

%% Test 2: Jacobian test
% test the geometric Jacobian matrix of the endeffector
% Jacobian_geom_b_end.m

% number of active joints
num_nja = finger_r.nja;

% geometric Jacobian computed by the class function 'Jacobian_geom_end'
J_class_b = finger_r.Jacobian_geom_b_end(q_r);% with respect to the base frame
W_R_b = finger_r.w_R_base();
J_class = blkdiag(W_R_b,W_R_b)*J_class_b;

% geometrical Jacobian using Jacobian_geom_mdh.m function
J_geom_func_b = Jacobian_geom_mdh(mdh_struct_to_matrix(finger_r.mdh_ori,1),q_r);
J_geom_func = blkdiag(W_R_b,W_R_b)*J_geom_func_b;

% geometric Jacobian computed by the rst toolbox
J_rst = geometricJacobian(rst_model,q_r,'endeffector'); % J_rst = [omega_x omega_y omega_z v_x v_y v_z]' 
J_rst = [J_rst(4:6,1:num_nja);J_rst(1:3,1:num_nja)]; % J_rst = [v_x v_y v_z omega_x omega_y omega_z]'

% validaiton
J_error = abs(J_class-J_rst);
J_error_2 = abs(J_geom_func-J_rst);
if max([J_error(:),J_error_2(:)]) > 1e-10
    fprintf('Test 2 (geometric Jacobian): failed! \n')
else
    fprintf('Test 2 (geometric Jacobian): pass! \n')
end

% analytical Jacobian
J_analytic_end_b = finger_r.Jacobian_analytic_b_end(q_r);
J_analytic_end = blkdiag(W_R_b,W_R_b)*J_analytic_end_b;

J_analytic_func_b = Jacobian_analytic_mdh(mdh_struct_to_matrix(finger_r.mdh_ori,1),q_r);
J_analytic_func = blkdiag(W_R_b,W_R_b)*J_analytic_func_b;



%% Test 3: Frame position test
% test properties: link.base_p & link.base_R

% Base to World frame transformation of class model 
W_p_b = finger_r.w_p_base();
W_R_b = finger_r.w_R_base();

test_3_error = 0;

for i = 1:finger_r.nl
    % rsi model: frame position & orientation in world frame
    T_rst_link_i = getTransform(finger_r.rst_model,q_r,finger_r.list_links(i).name);
    W_base_p_rsi = T_rst_link_i(1:3,4);
    W_base_R_rsi = T_rst_link_i(1:3,1:3);
    
    % class model: frame position & orientation in world frame
    W_base_p_class = W_R_b * finger_r.list_links(i).base_p + W_p_b;
    W_base_R_class = W_R_b* finger_r.list_links(i).base_R;
    
    base_p_error = abs(W_base_p_class-W_base_p_rsi);
    base_R_error = abs(W_base_R_class-W_base_R_rsi);
    
    
    if max(base_p_error(:)) > 1e-10 || max(base_R_error(:)) > 1e-10
        test_3_error = 1;
    end
end

% validaiton
if test_3_error == 1
    fprintf('Test 3 (Frame position): failed! \n')
else
    fprintf('Test 3 (Frame position): pass! \n')
end


%% Test 4: Contact test
% set the contact point at the end of the link (next frame)
% then compare the J_frame (rst) to the J_contact
% test: add_contact & Jacobian_geom_b_contact.m

% set contact point at the end of each link

if finger_r.nc == 0
    for i = 1:finger_r.nl-1
        b_R_i = finger_r.list_links(i).base_R;
        b_p_i1 = finger_r.list_links(i+1).base_p;
        b_p_i = finger_r.list_links(i).base_p;
        finger_r.list_links(i).add_contact(b_R_i'*(b_p_i1-b_p_i));
%         finger_r.list_links(i).add_contact([finger_r.list_links(i).Length 0 0]');
    end
    mdh_ori = mdh_struct_to_matrix(finger_r.mdh_ori(end,:), 1);
    T_end = T_mdh_multi(mdh_ori(end,:));
    finger_r.list_links(finger_r.nl).add_contact(T_end(1:3,4));
end
finger_r.update_list_contacts; % update link


% Test 4-1 geometric Jacobian test
W_R_b = finger_r.w_R_base();
J_class_contacts = [];

for i = 1:finger_r.nl
    if finger_r.list_links(i).nc
        for j = 1:finger_r.list_links(i).nc
            b_J_c_i = finger_r.Jacobian_geom_b_contact(q_r,finger_r.list_links(i).contacts(j));
            J_class_contacts = [J_class_contacts;blkdiag(W_R_b,W_R_b)*b_J_c_i];
        end
    end
end

J_rst_frame = [];

for i = 1:finger_r.nl
    J_rst_frame_i = geometricJacobian(rst_model,q_r,rst_model.BodyNames{i+1});
    J_rst_frame_i = [J_rst_frame_i(4:6,:);J_rst_frame_i(1:3,:)];
    J_rst_frame_i(:,i+2:end) = 0; % the contact is not influenced by the next joint 
    J_rst_frame = [J_rst_frame;J_rst_frame_i];
end


J_contact_error = abs(J_class_contacts-J_rst_frame);

if max(J_contact_error(:)) > 1e-10
    fprintf('Test 4-1 (contacts Jacobian): failed! \n')
else
    fprintf('Test 4-1 (contacts Jacobian): pass! \n')
end


show(rst_model,q_r,'Collisions','on','Visuals','off');
hold on
finger_r.print_contact()



% Test 4-2 delete all contacts
% delete_all_contacts()


% finger_r.delete_all_contacts();


% return
%% Test 5 inverse dynamic test

% random states
q_rD = rand(size(q_r));
q_rDD = rand(size(q_r));
F_ext = rand(6,1);


Tau_class = finger_r.invdyn_ne_w_end(q_r,q_rD,q_rDD,F_ext);


% transfer the external force exerting on the 
transform = getTransform(rst_model,q_r,'endeffector');
W_R_end = transform(1:3,1:3);
F_ext_rst = externalForce(rst_model,'endeffector',[W_R_end'*F_ext(4:6);W_R_end'*F_ext(1:3)],q_r);
Tau_rst = inverseDynamics(rst_model, q_r,q_rD,q_rDD, F_ext_rst);


Tau_error = abs(Tau_class-Tau_rst);

if max(Tau_error(:)) > 1e-10
    fprintf('Test 5 (inverse dynamic): failed! \n')
else
    fprintf('Test 5 (inverse dynamic): pass! \n')
end

%% Test 6 Forward dynamic test

% random states
q_rD = rand(size(q_r));
tau = rand(size(q_r));
F_ext = [zeros(6,finger_r.nj+1),rand(6,1)];


qDD_class = finger_r.fordyn_ne_w_end(q_r,q_rD,tau,F_ext);


% transfer the external force exerting on the 
transform = getTransform(rst_model,q_r,'endeffector');
W_R_end = transform(1:3,1:3);
F_ext_rst = externalForce(rst_model,'endeffector',[W_R_end'*F_ext(4:6,end);W_R_end'*F_ext(1:3,end)],q_r);
qDD_rst = forwardDynamics(rst_model,q_r,q_rD,tau,F_ext_rst);

qDD_error = abs(qDD_class-qDD_rst);

if max(qDD_error(:)) > 1e-10
    fprintf('Test 6 (forward dynamic): failed! \n')
else
    fprintf('Test 6 (forward dynamic): pass! \n')
end




%% Test 7 Tendon moment arm properties
% poly 3 function: MA = ax^3+bx^2+cx+d

if finger_r.nt == 0
    finger_r.add_tendon('Flex_1', [1,1,1,1]);
    finger_r.add_tendon('Flex_2', [1,1,1,0]);
    finger_r.add_tendon('Flex_3', [1,1,0,0]);
    finger_r.add_tendon('Ext_1', [-1,-1,-1,-1]);
end
q_r = rand(4,1);
finger_r.update_finger(q_r);
finger_r.M_coupling;
finger_r.set_tendon_par_MA_poly3(3,1,[0,0,0,0.03]);
finger_r.set_tendon_par_MA_poly3(3,2,[0,0,0,0.035]);
finger_r.M_coupling;


%% Test 8: inverse kinematic

p_link_all_w_r = finger_r.get_p_all_links;
figure(1)
plot3(finger_r.w_p_base(1),finger_r.w_p_base(2),finger_r.w_p_base(3),'x','MarkerSize',15);
hold on
plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','r');
hold on
grid on
axis equal


x_init = p_link_all_w_r(:,end);
x_des = x_init - 0.5*rand(3,1);
plot3(x_des(1),x_des(2),x_des(3),'*','Color','b','MarkerSize',20);
hold on


iter_max = 100;
alpha = 0.9;
color_plot = [1,0,0];
tol = 1e-9;
[q,q_all,x_res,phi_x,iter] = finger_r.invkin_trans_numeric(x_des,iter_max,tol,alpha);

p_link_all_w_r = finger_r.get_p_all_links;
plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','c');

%% Test 9: inverse kinematic with joint limits


q_init = rand(4,1);
finger_r.w_p_base = rand(3,1);
finger_r.w_R_base = euler2R_XYZ(rand(1,3));
finger_r.update_finger(q_init);

p_link_all_w_r = finger_r.get_p_all_links;
figure(1)
plot3(finger_r.w_p_base(1),finger_r.w_p_base(2),finger_r.w_p_base(3),'x','MarkerSize',15);
hold on
% plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','--','Color','r');
% hold on
grid on
axis equal

% update joint limits
q_limit = [-30,30;0,80;0,80;0,50]*pi/180;
for i = 1:finger_r.nj
    finger_r.list_joints(i).q_limits = q_limit(i,:);
end
finger_r.update_joints_info;



x_init = p_link_all_w_r(:,end);
x_des = x_init - 0.01*rand(3,1);
plot3(x_des(1),x_des(2),x_des(3),'.','Color','r','MarkerSize',40);
hold on


iter_max = 1000;
alpha = 0.8;
color_plot = [1,0,0];
tol = 1e-9;
[q,q_all,x_res,phi_x,iter] = finger_r.invkin_trans_numeric_joint_limits(x_des,iter_max,tol,alpha);
finger_r.update_finger(q_init);
hold on
[q_mex,q_all_mex,x_res_mex,phi_x_mex,iter_mex] = finger_r.invkin_trans_numeric_joint_limits(x_des,iter_max,tol,alpha,1);
finger_r.print_finger('k');
hold on 
% mdh_matrix_ik9 = mdh_struct_to_matrix(finger_r.mdh_ori, 1);
% [q_k,q_all_k,x_res_k,phi_x_k,iter_k] = ik_trans_numeric_joint_limits(mdh_matrix_ik9,...
%                         x_des,q_init,finger_r.limits_q(:,1:2),iter_max,tol,alpha);


% p_link_all_w_r = finger_r.get_p_all_links;
% finger_r.print_finger('c');
% hold on 
% finger_r.update_finger(q_k);
% finger_r.print_finger('b');

% q_error = abs(q-q_k);
q_error_mex = abs(q-q_mex);

if max([q_error_mex(:)]) > 1e-4
    fprintf('Test 9 (ik with joint limits): failed! \n')
else
    fprintf('Test 9 (ik with joint limits): pass! \n')
end

return
%% Workspace


q_limit = rand(finger_r.nja,2);
q_limit(:,1) = - q_limit(:,1);

[q1,q2,q3,q4] = ndgrid(q_limit(1,1):0.05:q_limit(1,2),...
    q_limit(2,1):0.3:q_limit(2,2),...
    q_limit(3,1):0.3:q_limit(3,2),...
    q_limit(4,1):0.3:q_limit(4,2));

figure(2)
for i = 1:size(q1,1)
    for j = 1:size(q1,2)
        for k = 1:size(q1,3)
            for h = 1:size(q1,4)
                q_i = [q1(i,j,k,h);q2(i,j,k,h);q3(i,j,k,h);q4(i,j,k,h)];
                finger_r.update_finger(q_i);
                p_link_all_w_r = finger_r.get_p_all_links;
                plot3(p_link_all_w_r(1,end),p_link_all_w_r(2,end),p_link_all_w_r(3,end),'.','Color','r');
                hold on
                
            end
         end
    end
end

grid on
axis equal