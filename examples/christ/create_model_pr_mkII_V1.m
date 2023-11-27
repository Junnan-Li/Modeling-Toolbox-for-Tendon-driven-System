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
robot_dimension = [a5,0,0,0]; % in meter

prost_low = Finger('Index', 'type','R_RRRR', 'l_links',robot_dimension(1:end));
mdh_default_struct = prost_low.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1); % alpha,a,theta,d
mdh_matrix(1,2) = a4;
mdh_matrix(1,3) = pi;
mdh_matrix(1,4) = d4;
mdh_matrix(2,1) = pi/2;
mdh_matrix(2,2) = a5;
mdh_matrix(2,3) = pi/2;
mdh_matrix(2,4) = d5;
mdh_matrix(3,1) = pi/2;
mdh_matrix(3,3) = pi/2;
mdh_matrix(4,1) = -pi/2;

prost_low.set_mdh_parameters(mdh_matrix(1:end,:));


T = T_mdh_multi(mdh_matrix(1:1,:));
T_base = T_mdh_multi([-pi/2 0 0 0]);
R_base = T_base(1:3,1:3); 

%% set states
prost_low.w_p_base = [0;0;0];
prost_low.w_R_base = euler2R_XYZ([-pi/2;-pi;pi/2]);

% init joint configurations
q_0 = [0,0,0,0]';
% udpate finger with given joint configurations
prost_low.update_finger(q_0);

%% set dynamic parameters

[m_p, rSges_p, mrSges_p, Ifges_p, Icges_p] = read_SW_inertia_chris(strcat(path_prosthesis, '/00_lib/mechanical parameters'),  ...
            {'T_el', 'T_fa', 'T_w1', 'T_w2_prencilia'});

        
% here they are changed compared to alex original model (based on new
com_converted = (eul2rotm([0,0,0])*rSges_p')';

% load new dynamic data
m_new = m_p;

% % SOFTHAND
% m_new(8) = 0.424;
% com_converted(end,1) = 0.07632;
% 
% % Prencilia Azzure
% m_new(8) = 0.7;
% com_converted(end,1) = 0.08530

% add base information
prost_low.set_base_dynpar(m_new(1),rSges_p(1,:)',Icges_p(1,:)' );

% update prost_low
for i = 1:prost_low.nl
    prost_low.list_links(i).set_mass(m_new(i));
    prost_low.list_links(i).set_inertia(Icges_p(i,:)); % regarding com
    prost_low.list_links(i).set_com(com_converted(i,:)'); %
end
prost_low.update_finger_par_dyn;


% Tau = prost_low.invdyn_ne_w_end_sym(1)
prost_low.plot_finger;