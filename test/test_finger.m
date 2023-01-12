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
finger_r = Finger('Index', 'RRRR', [0.5 0.3 0.2]);
finger_s = Finger('Index', 'RRRs', rand(1,3));
finger_p = Finger('Index', 'RRRp', rand(1,3));

finger_list = {finger_r,finger_s,finger_p};

finger_r.w_p_base = [1,0,0]';
finger_r.w_R_base = euler2R_XYZ([0 0 pi/2]);

% joint configurations
% q = [45,0,0]'*pi/180;
q_r = rand(4,1);
% q_r = [pi/4<,pi/3,0,0]';
q_s = rand(4,1)*pi;
q_p = rand(4,1)*pi;

% udpate finger with given joint configurations
finger_r.update_finger(q_r);
% finger_r.update_rst_model;

% load rst model from finger class
rst_model = finger_r.rst_model;

% plot rst model
show(rst_model,q_r,'Collisions','on','Visuals','off');

% return

%% Test 1:  mdh parameters test
% disp('Test start: Transformation matrix test')

% mdh parameter from class properties
mdh = mdh_struct_to_matrix(finger_r.mdh,1);
b_T_class = T_mdh_multi(mdh);
W_T_b = finger_r.get_W_T_B();
T_class = W_T_b * b_T_class;

% mdh parameters from rst model
T_rst = getTransform(rst_model,[q_r],'endeffector');


T_error = abs(T_class-T_rst);
% assert(max(T_error(:)) < 1e-12, 'Class Error: Transformation matrix from mdh parameters do not match!')
if max(T_error(:)) > 1e-10
    fprintf('Test 1 (Transformation matrix): failed! \n')
else
    fprintf('Test 1 (Transformation matrix): pass! \n')
end

%% Test 2: geometric Jacobian test

% number of active joints
num_nja = finger_r.nja;

% geometric Jacobian computed by the class function 'Jacobian_geom_end'
J_class_b = finger_r.Jacobian_geom_b_end(q_r);
W_R_b = finger_r.w_R_base();
J_class = [W_R_b*J_class_b(1:3,:);W_R_b*J_class_b(4:6,:)];


% geometric Jacobian computed by the rst toolbox
J_rst = geometricJacobian(rst_model,q_r,'endeffector'); % J_rst = [omega_x omega_y omega_z v_x v_y v_z]' 
J_rst = [J_rst(4:6,1:num_nja);J_rst(1:3,1:num_nja)]; % J_rst = [v_x v_y v_z omega_x omega_y omega_z]'

J_error = abs(J_class-J_rst);

if max(J_error(:)) > 1e-10
    fprintf('Test 2 (geometric Jacobian): failed! \n')
else
    fprintf('Test 2 (geometric Jacobian): pass! \n')
end
%% Test 3: Frame position test
% Links.base_p & Links.base_R

% Base to World frame transformation of class model 
W_p_b = finger_r.w_p_base();
W_R_b = finger_r.w_R_base();

test_3_error = 0;

for i = 1:finger_r.nl
    % rsi model: frame position & orientation in world frame
    T_rst_link_i = getTransform(finger_r.rst_model,q_r,finger_r.list_links(i).name);
    base_p_rsi = T_rst_link_i(1:3,4);
    base_R_rsi = T_rst_link_i(1:3,1:3);
    
    % class model: frame position & orientation in world frame
    base_p_class = W_R_b * finger_r.list_links(i).base_p + W_p_b;
    base_R_class = W_R_b* finger_r.list_links(i).base_R;
    
    base_p_error = abs(base_p_class-base_p_rsi);
    base_R_error = abs(base_R_class-base_R_rsi);
    
    
    if max(base_p_error(:)) > 1e-10 || max(base_R_error(:)) > 1e-10
        test_3_error = 1;
    end

%     assert(max(base_p_error(:)) < 1e-12, 'Class Error: link position do not match!')
%     assert(max(base_R_error(:)) < 1e-12, 'Class Error: link rotation do not match!')

end

if test_3_error == 1
    fprintf('Test 3 (Frame position): failed! \n')
else
    fprintf('Test 3 (Frame position): pass! \n')
end

return
%% Test 4: Geometric Jacobian of contact test

disp('Test start: link kinematic test')

q_r = rand(4,1)*pi;

for i = 1:finger_r.nl
    finger_r.list_links(i).add_contact([finger_r.list_links(i).Length/2 0 0]')
end
finger_r.update_all_contacts;
J_c = [];

for i = 1:finger_r.nl
    if finger_r.list_links(i).nc
        for j = 1:finger_r.list_links(i).nc
            J_c = [J_c;finger_r.Jacobian_geom_contact(q_r,finger_r.list_links(i).contacts(j))];
        end
    end
end


disp('Test pass!')
