% finger class function test



clear all
close all
clc


% define different types of fingers
finger_r = Finger('Index', 'RRRR', [0.5 0.3 0.2]);
finger_s = Finger('Index', 'RRRs', rand(1,3));
finger_p = Finger('Index', 'RRRp', rand(1,3));

finger_list = {finger_r,finger_s,finger_p};



finger_r.w_p_base = [1,0,0]';
finger_r.w_R_base = euler2R_XYZ([0 0 pi/6]);


% joint configurations
% q = [45,0,0]'*pi/180;
q_r = rand(4,1)*pi;
q_s = rand(4,1)*pi;
q_p = rand(4,1)*pi;

% udpate finger with given joint configurations
finger_r.update_finger(q_r);
% finger_r.update_rst_model;

% load rst model from finger class
rst_model = finger_r.rst_model;

% plot rst model
show(rst_model,[q_r;0],'Collisions','on','Visuals','off');

%% mdh parameters test
disp('Test start: Transformation matrix test')

% mdh parameter from class properties
mdh = mdh_struct_to_matrix(finger_r.mdh,1);
T_class = T_mdh_multi(mdh);

% mdh parameters from rst model
T_rst = getTransform(rst_model,[q_r;0],'endeffector');


T_error = abs(T_class-T_rst);
assert(max(T_error(:)) < 1e-12, 'Class Error: Transformation matrix from mdh parameters do not match!')

disp('Test pass!')

%% geometric Jacobian test

disp('Test start: geometric Jacobian test')
% number of active joints
num_nja = finger_r.nja;

% geometric Jacobian computed by the class function 'Jacobian_geom_end'
J_class = finger_r.Jacobian_geom_end(q_r);

% geometric Jacobian computed by the rst toolbox
J_rst = geometricJacobian(rst_model,[q_r;0],'endeffector'); % J_rst = [omega_x omega_y omega_z v_x v_y v_z]' 
J_rst = [J_rst(4:6,1:num_nja);J_rst(1:3,1:num_nja)]; % J_rst = [v_x v_y v_z omega_x omega_y omega_z]'

J_error = abs(J_class-J_rst);

assert(max(J_error(:)) < 1e-12, 'Class Error: Geometric Jacobian do not match!')
disp('Test pass!')

%% link kinematic peoperties test
% Links.base_p & Links.base_R


disp('Test start: link kinematic test')

q_r = rand(4,1)*pi;

% udpate finger with given joint configurations
finger_r.update_finger(q_r);
finger_r.update_rst_model;


for i = 1:finger_r.nl

    T_rst_link_i = getTransform(finger_r.rst_model,[q_r;0],finger_r.list_links(i).name);
    base_p_rsi = T_rst_link_i(1:3,4);
    base_R_rsi = T_rst_link_i(1:3,1:3);
    
    base_p_class = finger_r.list_links(i).base_p;
    base_R_class = finger_r.list_links(i).base_R;
    
    base_p_error = abs(base_p_class-base_p_rsi);
    base_R_error = abs(base_R_class-base_R_rsi);
    
    assert(max(base_p_error(:)) < 1e-12, 'Class Error: link position do not match!')
    assert(max(base_R_error(:)) < 1e-12, 'Class Error: link rotation do not match!')

end

disp('Test pass!')

%% Geometric Jacobian of contact test

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
