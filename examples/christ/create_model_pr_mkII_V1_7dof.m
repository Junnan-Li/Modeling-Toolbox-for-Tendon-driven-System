% Create prosthesis model 
% from Christ data
% 19.10.2023 Junnan Li

%clear all 
%close all
%clc

%% modified DH parameters pr_mkII_V1
a4 = 0.363;
a5 = 0.0;
d4 = 0.149; % new
% d5 = 0.25686;
d5 = 0.29696; % new

%% define the prosthesis
robot_dimension = [0,0,a4,a5,0,0,0]; % in meter

prost = Finger('Index', 'type','R_RRRRRRR', 'l_links',robot_dimension); % entire prosthetic (7 dof)
prost_low = Finger('Index', 'type','R_RRRR', 'l_links',robot_dimension(4:end)); % only low part (4 dof) -> elbow, forearm, wrist
mdh_default_struct = prost.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1); % alpha,a,theta,d
mdh_matrix(1,1) = -pi/2;
mdh_matrix(1,3) = pi/2;
mdh_matrix(2,1) = -pi/2;
mdh_matrix(2,3) = pi/2;
mdh_matrix(3,1) = -pi/2;
mdh_matrix(4,2) = a4;
mdh_matrix(4,3) = pi;
mdh_matrix(4,4) = d4;
mdh_matrix(5,1) = pi/2;
mdh_matrix(5,2) = a5;
mdh_matrix(5,3) = pi/2;
mdh_matrix(5,4) = d5;
mdh_matrix(6,1) = pi/2;
mdh_matrix(6,3) = pi/2;
mdh_matrix(7,1) = -pi/2;


prost.set_mdh_parameters(mdh_matrix);
prost_low.set_mdh_parameters(mdh_matrix(4:end,:));


T = T_mdh_multi(mdh_matrix(1:1,:));
T_base = T_mdh_multi([-pi/2 0 0 0]);
R_base = T_base(1:3,1:3); 

%% set states
% set base position and orientation
prost.w_p_base = [-0.1131;-0.2539; -0.2539];
prost.w_R_base = euler2R_XYZ([pi/2;0;0]);
% prost.w_R_base = R_base;

% init joint configurations
q_0 = [0,0,0,0,0,0,0]';
% udpate finger with given joint configurations
prost.update_finger(q_0);

% load rst model from finger class
% prost.update_rst_model;
% rst_model_r = prost.rst_model;

% 
% figure(1)
% % rst_model_r.show(q_0_rst);
% % hold on
% par_plot = prost.plot_parameter_init;
% par_plot.axis_len = 0.1;
% prost.plot_finger(par_plot);
% grid on 
% axis equal

prost_low.w_p_base = [0;0;0];
%prost_low.w_R_base = euler2R_XYZ([pi/2;0;0]);
prost_low.w_R_base = euler2R_XYZ([-pi/2;-pi;pi/2]);
% prost.w_R_base = R_base;

% init joint configurations
q_0 = [0,0,0,0]';
% udpate finger with given joint configurations
prost_low.update_finger(q_0);
%par_plot = prost_low.plot_parameter_init;
%par_plot.axis_len = 0.1;
%prost_low.plot_finger(par_plot);
%axis equal
% return
%% set dynamic parameters

% CAREFUL HERE!!!!
% This doesnt match at all as alex is in g and m and mine in m and kg

% load examples\christ\Junnan\02_calibweight\mech_param.mat
%path___ = path_prosthesis'/home/chris/repos/hand_modeling_toolbox-main/examples/christ/Junnan/02_calibweight/';
[m_p, rSges_p, mrSges_p, Ifges_p, Icges_p] = read_SW_inertia(strcat(path_prosthesis, '/00_lib/mechanical parameters'),  ...
            {'T_0_0', 'T_0_sh1', 'T_0_sh2','T_0_sh3', 'T_el', 'T_fa', 'T_0_wj1', 'T_wj'});


% here they are changed compared to alex original model (based on new
% measueemtents by chris in the solidowkrs file)
% com_converted = [rSges_p(:,3), rSges_p(:,2), rSges_p(:,1)];
com_converted = (eul2rotm([0,0,0])*rSges_p')';
% load new dynamic data
m_new = m_p;
m_new(5) = 0.5959;
m_new(6) = 0.52;

% SOFTHAND
m_new(8) = 0.424;
com_converted(end,1) = 0.07632;

% Prencilia Azzure
m_new(8) = 0.7;
com_converted(end,1) = 0.08530



for i = 2:length(m_new)
    prost.list_links(i-1).set_mass(m_new(i));
    prost.list_links(i-1).set_inertia(Icges_p(i,:)); % regarding com
    prost.list_links(i-1).set_com(com_converted(i,:)'); %
end

% add base information
prost.set_base_dynpar(m_new(1),rSges_p(1,:)',Icges_p(1,:)' );
% prost.set_g_w([0,0,-9.81]');
prost.update_finger_par_dyn;

% update prost_low
for i = 1:prost_low.nl
    prost_low.list_links(i).set_mass(m_new(4+i));
    prost_low.list_links(i).set_inertia(Icges_p(4+i,:)); % regarding com
    prost_low.list_links(i).set_com(com_converted(4+i,:)'); %
end
prost_low.update_finger_par_dyn;



% Tau = prost_low.invdyn_ne_w_end_sym(1)
prost.plot_finger;
prost_low.plot_finger;
return



%% alex code for plot 

% parameters for alex code
a4 = 0.363;
a5 = 0.0;
d4 = .149; % new
% d5 = 0.25686;
d5 = 0.29696; % new
t1 = [a4; a5; d4; d5;];
pkin_p = t1 ;

s_mp_in = fct_param_boxes_B01_wr_desktop(); % paylod 1.
frame_index_order = {'T_0_0', 'T_0_sh1', 'T_0_sh2','T_0_sh3', 'T_0_el', 'T_0_fa', 'T_0_wj1', 'T_0_wj2'};
s_mp_out = fct_calc_mass_parameter_box(s_mp_in);

% plot position
for i = 1:1
q0 = [0,0,0,0,0,0,0]';
q0 = rand(7,1);
T_c_mdh = exoprB01_fkine_fixb_rotmat_mdh_sym_varpar(q0, pkin_p);

R_W_0 = eul2rotm([pi/2;0;0]','XYZ');
T_W_0 = [R_W_0, prost.w_p_base; 0, 0, 0, 1];
s_plot_box_edges.TF.KS0.T_0_0.data = eye(4);
s_plot_box_edges.TF.KS0.T_0_sh1.data =  T_c_mdh(:,:,2);
s_plot_box_edges.TF.KS0.T_0_sh2.data =  T_c_mdh(:,:,3);
s_plot_box_edges.TF.KS0.T_0_sh3.data =  T_c_mdh(:,:,4);
s_plot_box_edges.TF.KS0.T_0_el.data =  T_c_mdh(:,:,5);
s_plot_box_edges.TF.KS0.T_0_fa.data =  T_c_mdh(:,:,6);
s_plot_box_edges.TF.KS0.T_0_wj1.data = T_c_mdh(:,:,7);
s_plot_box_edges.TF.KS0.T_0_wj2.data = T_c_mdh(:,:,8);
s_plot_box_edges.TF.KSW.T_W_0.data = T_W_0;
s_plot_edges_out = fct_calc_box_edges(s_mp_out, s_plot_box_edges);

figure(1);
fct_plot_boxes(s_plot_edges_out, s_mp_out)
view(22.5,35.6)
xlabel('x')
ylabel('y')
zlabel('z')
hold on
grid on;
axis equal



% plot prosthesis of model
prost.update_finger(q0);
par_plot = prost.plot_parameter_init;
par_plot.axis_len = 0.1;

prost.plot_finger
par_plot_com = par_plot;
par_plot_com.markersize = 28;
par_plot_com.markercolor = 'k';

% prost.list_links(3).set_com([0,0,0]);
% prost.update_finger_par_dyn;
prost.plot_com(par_plot_com);
prost_low.plot_finger
par_plot_com = par_plot;
par_plot_com.markersize = 28;
par_plot_com.markercolor = 'k';

% prost.list_links(3).set_com([0,0,0]);
% prost.update_finger_par_dyn;
prost_low.plot_com(par_plot_com);
hold off
end
return

%% tmp test

% 
q_sym = sym('q',[1 7])

Tau = prost.invdyn_ne_w_end( q_sym, zeros(7,1), zeros(7,1), zeros(6,1));
% 
% 
% [qDD,M_fd,C_fd,G_fd] = prost.fordyn_ne_w_end( zeros(7,1), qJD, zeros(7,1), zeros(6,9),0);


%% setup simulink 

q_init = zeros(7,1);
qD_init = zeros(7,1);

%% forward dynamic and animation

% simulation settings
sim_time = 5; % s
dt = 1e-2; % s
step = sim_time/dt;


% state variables init
n_q = prost.nj;
q0 = zeros(n_q,1);
qd0 = zeros(n_q,1);

state = zeros(2*n_q,step); % [q;qd]
state(:,1) = [q0;qd0];
state_d = zeros(2*n_q,step);
T = zeros(step,1);

% setting
Tau = zeros(n_q,step);
F_ext = zeros(6,n_q+2);
% prost.compile_functions;
mex = 1;

h = figure(2)
prost.update_finger(q0);
prost.plot_finger(par_plot);
% hold off
axis equal
xlim([-1,1])
ylim([-1,1])
zlim([-1,1])
view([15 10 5])
T = zeros(step,1);
for i = 1:step
    tic;
    
    state_i = state(:,i);
    q_i = state_i(1:n_q);
    qd_i = state_i(n_q+1:end);
%     q_theta_i = q_desired - q_i;
%     qd_theta_i = qd_desired - qd_i;
%     

    Tau_i = Tau(:,i);
    
    [qdd_i,M_fd,C_fd,G_fd] = prost.fordyn_ne_w_end(q_i, qd_i, Tau_i, F_ext, mex);
    
    %     stated_i = rungekutta4(@(t_i,state_i)finger_r.fordyn(t_i ,state_i, Tau_i, F_ext),delta_t,t_i,state_i);
    %     state(:,i+1) = delta_t.*stated_i + state_i;
    %     state_d(:,i) = stated_i;
%     
    state_d(1:n_q,i) = qd_i;
    state_d(n_q+1:end,i) = qdd_i;
    
    qd_i_next = qd_i + dt * qdd_i/2;
    q_i_next = q_i + dt * qd_i_next/2;

    state(:,i+1) =  [q_i_next;qd_i_next];
    T(i+1) = T(i)+dt;
    
    % update plot 
    cla
    prost.update_finger(q_i_next);
    prost.plot_finger(par_plot);
    prost.plot_com(par_plot_com);
% fct_plot_boxes(s_plot_edges_out, s_mp_out)

% xlabel('x')
% ylabel('y')
% zlabel('z')
% hold on
% grid on;
% axis equal

    drawnow
    
end


