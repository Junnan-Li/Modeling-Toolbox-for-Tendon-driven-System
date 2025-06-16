


clear all
close all
clc

% format short
%% Robot Definition
deg = pi/180;
mm = 1e-3;
        
% Define links from: https://github.com/petercorke/robotics-toolbox-matlab/blob/master/models/mdl_panda.m
L1 = RevoluteMDH('a',     0.0, 'd', 0.333, 'alpha',   0.0, 'qlim', [-2.8973 2.8973], ...
    'm', 4.970684, 'r', [3.875e-03 2.081e-03 0], 'I', [7.03370e-01  7.06610e-01  9.11700e-03 -1.39000e-04  1.91690e-02  6.77200e-03], 'G', 1);
L2 = RevoluteMDH('a',     0.0, 'd',   0.0, 'alpha', -pi/2, 'qlim', [-1.7628 1.7628], ...
    'm', 0.646926, 'r', [-3.141e-03 -2.872e-02 3.495e-03], 'I', [7.96200e-03  2.81100e-02  2.59950e-02 -3.92500e-03  7.04000e-04  1.02540e-02], 'G', 1);
L3 = RevoluteMDH('a',     0.0, 'd', 0.316, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], ...
    'm', 3.228604, 'r', [ 2.7518e-02 3.9252e-02 -6.6502e-02], 'I', [3.72420e-02  3.61550e-02  1.08300e-02 -4.76100e-03 -1.28050e-02 -1.13960e-02], 'G', 1);
L4 = RevoluteMDH('a',  0.0825, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-3.0718 -0.0698], ...
    'm', 3.587895, 'r', [-5.317e-02 1.04419e-01 2.7454e-02], 'I', [2.58530e-02  1.95520e-02  2.83230e-02  7.79600e-03  8.64100e-03 -1.33200e-03], 'G', 1);
L5 = RevoluteMDH('a', -0.0825, 'd', 0.384, 'alpha', -pi/2, 'qlim', [-2.8973 2.8973], ...
    'm', 1.225946, 'r', [-1.1953e-02 4.1065e-02 -3.8437e-02], 'I', [3.55490e-02  2.94740e-02  8.62700e-03 -2.11700e-03  2.29000e-04 -4.03700e-03], 'G', 1);
L6 = RevoluteMDH('a',     0.0, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-0.0175 3.7525], ...
    'm', 1.666555, 'r', [6.0149e-02 -1.4117e-02 -1.0517e-02], 'I', [1.96400e-03  4.35400e-03  5.43300e-03  1.09000e-04  3.41000e-04 -1.15800e-03], 'G', 1);
L7 = RevoluteMDH('a',   0.088, 'd',   0.107, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], ...
    'm', 7.35522e-01, 'r', [1.0517e-02 -4.252e-03 6.1597e-02 ], 'I', [1.25160e-02  1.00270e-02  4.81500e-03 -4.28000e-04 -7.41000e-04 -1.19600e-03], 'G', 1);

% L1 = RevoluteMDH('a',     0.0, 'd', 0.333, 'alpha',   0.0, 'qlim', [-2.8973 2.8973], ...
%     'm', 4.9707, 'r', [3.9e-03 2.1e-03 0], 'I', [7.034e-01  7.066e-01  9.1e-03 -1.39000e-04  1.92e-02  6.8e-03], 'G', 1);
% L2 = RevoluteMDH('a',     0.0, 'd',   0.0, 'alpha', -pi/2, 'qlim', [-1.7628 1.7628], ...
%     'm', 0.6469, 'r', [-3.1e-03 -2.87e-02 3.5e-03], 'I', [7.96200e-03  2.81100e-02  2.59950e-02 -3.92500e-03  7.04000e-04  1.02540e-02], 'G', 1);
% L3 = RevoluteMDH('a',     0.0, 'd', 0.316, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], ...
%     'm', 3.2286, 'r', [ 2.75e-02 3.93e-02 -6.65e-02], 'I', [3.72420e-02  3.61550e-02  1.08300e-02 -4.76100e-03 -1.28050e-02 -1.13960e-02], 'G', 1);
% L4 = RevoluteMDH('a',  0.0825, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-3.0718 -0.0698], ...
%     'm', 3.5879, 'r', [-5.32e-02 1.044e-01 2.75e-02], 'I', [2.590e-02  1.960e-02  2.830e-02  7.800e-03  8.64100e-03 -1.33200e-03], 'G', 1);
% L5 = RevoluteMDH('a', -0.0825, 'd', 0.384, 'alpha', -pi/2, 'qlim', [-2.8973 2.8973], ...
%     'm', 1.2259, 'r', [-1.12e-02 4.11e-02 -3.84e-02], 'I', [3.550e-02  2.950e-02  8.600e-03 -2.100e-03  2.29000e-04 -4.000e-03], 'G', 1);
% L6 = RevoluteMDH('a',     0.0, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-0.0175 3.7525], ...
%     'm', 1.6666, 'r', [6.01e-02 -1.41e-02 -1.05e-02], 'I', [2.00e-03  4.400e-03  5.400e-03  1.09000e-04  3.41000e-04 -1.200e-03], 'G', 1);
% L7 = RevoluteMDH('a',   0.088, 'd',   0.107, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], ...
%     'm', 7.355e-01, 'r', [1.05e-02 -4.3e-03 6.16e-02 ], 'I', [1.25e-02  1.00e-02  4.800e-03 -4.28000e-04 -7.41000e-04 -1.2e-03], 'G', 1);


% Create SerialLink object
panda = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'PANDA', 'manufacturer', 'Franka-Emika', 'tool', transl([0 0 110*mm]));
panda.gravity = [0, 0, -9.80665];




%% 
franka = create_finger_random('Franka', 7);

mdh_default_struct = franka.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix = [0,     0,   0,0.333;...
              -pi/2, 0,   0, 0;...
              pi/2,  0,   0, 0.316;...
              pi/2,  0.0825, 0, 0; ...
              -pi/2, -0.0825, 0, 0.384;...
              pi/2,  0,   0, 0;...
              pi/2,  0.088, 0, 0.107;
              0,     0,     0, 0.11];


franka.set_mdh_parameters(mdh_matrix);
franka.set_base([0,0,0]',eye(3))

plot_par = franka.plot_parameter_init();

for i = 1:7
    franka.list_links(i).set_mass((panda.links(i).m));
    franka.list_links(i).set_com(panda.links(i).r);

    Ixx = panda.links(i).I(1,1);
    Iyy = panda.links(i).I(2,2);
    Izz = panda.links(i).I(3,3);
    Ixy = panda.links(i).I(1,2);
    Iyz = panda.links(i).I(2,3);
    Ixz = panda.links(i).I(1,3);
    % in sequence of [xx yy zz yz xz xy]
    franka.list_links(i).set_inertia([Ixx Iyy Izz Iyz Ixz Ixy]);
end

franka.update_finger_par_dyn;
franka.set_g_w(panda.gravity);

q_init = rand(1,7);
franka.update_finger(q_init');
franka.plot_finger(plot_par)

grid on
axis equal

% q_init  = [0 0 0 0 0 0 0]*deg;
panda.plot(q_init)

[Ts,T_all] = panda.fkine(q_init);
T_ee = franka.get_T_all_links;

for i = 1:7
    t_pc_i = T_all(i).T;
    T_link_i = T_ee(:,:,i+1);
    error_T = t_pc_i-T_link_i;
    if max(abs(error_T(:))) > 1e-6
        error('kinematic wrong')
    end
end


% figure(2)
% franka.plot_finger(plot_par)
% franka.plot_com(plot_par)
% grid on 
% axis equal
%% dynamic parameters
q = rand(1,7);
q(4)=0;
qd = rand(1,7);
M_peter = panda.inertia(q);
G_peter = panda.gravload(q);
C = panda.coriolis(q, qd) * qd';
franka.update_finger(q');

[qDD,M_fd,C_fd,G_fd] = franka.fordyn_ne_w_end(q', qd',zeros(franka.nj,1), zeros(6,franka.nj+2));
% M_fd

MassMatrix = get_MassMatrix(q);
g_li_hat = get_GravityVector(q);
return
%% sym

q_r_sym = sym('q',[franka.nj,1], 'real');
q_rd_sym = sym('qd',[franka.nj,1], 'real');
mdh_default_struct = franka.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
x_base = zeros(6,1);
var_name = {q_r_sym,q_rd_sym};

[Tau, M, C, G] = invdyn_lag_mdh_sym(q_r_sym,q_rd_sym,sym(zeros(franka.nj,1)), sym(mdh_matrix),sym(x_base), sym(franka.par_dyn_f.mass_all),...
                        sym(franka.par_dyn_f.com_all), sym(franka.par_dyn_f.inertia_all), sym(franka.par_dyn_f.g)); %

func_name = strcat('output/invdyn_lag_mdh_sym_', franka.name);
matlabFunction(M,"File",strcat(func_name,'_M'),...
        "Vars", var_name);

func_name = strcat('output/invdyn_lag_mdh_sym_', franka.name);
matlabFunction(G,"File",strcat(func_name,'_G'),...
        "Vars", var_name);
func_name = strcat('output/invdyn_lag_mdh_sym_', franka.name);
matlabFunction(C,"File",strcat(func_name,'_C'),...
        "Vars", var_name);
return
%% 
Franka_robot= Hand('franka');
Franka_robot.add_finger(franka);

Franka_robot.update_hand_par_dyn;
Franka_robot.set_g_w(panda.gravity);
Franka_robot.update_sim_par;
%% 

q_init = rand(1,7);
Franka_robot.update_hand(q_init');
Franka_robot.plot_hand(plot_par)

grid on
axis equal

% q_init  = [0 0 0 0 0 0 0]*deg;
panda.plot(q_init)

[Ts,T_all] = panda.fkine(q_init);
T_ee = Franka_robot.get_w_T_all;

for i = 1:7
    t_pc_i = T_all(i).T;
    T_link_i = T_ee(:,:,i+1);
    error_T = t_pc_i-T_link_i
    if max(abs(error_T(:))) > 1e-6
        error('kinematic wrong')
    end
end


figure(2)
Franka_robot.plot_hand(plot_par)
Franka_robot.plot_hand_com(plot_par)
grid on 
axis equal
%% dynamic parameters
q = rand(1,7);
qd = rand(1,7);
M_peter = panda.inertia(q);
G_peter = panda.gravload(q);
C = panda.coriolis(q, qd) * qd';
Franka_robot.update_hand(q');
[qDD,M,C,G] = Franka_robot.fordyn_ne_hand_w_end(q',qd');


Franka_robot.create_sim_functions_hand
