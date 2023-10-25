% Alexander Toedtheide Feb 2020
% init for 

% filepath_ = fileparts(fileparts(fileparts(fileparts(( mfilename('fullpath') )))));
% path___ = fullfile(filepath_, '00_lib','mechanical_parameters','02');

%%

NJ = 7;
NQJ_p = NJ;
IND_NO_WR = 1:NJ-2;
IND_WR = NJ-1:NJ;
IND_ACT = NJ+1;
IND_WR_FLP = [NJ, NJ-1];

%% Initial joint conditions
NQJ = 7;
qJ_t0_p = [0 0 0 0 0 0 0]';
qJD_t0_p = zeros(NQJ,1);
g_world = [0;0;-9.81];
g_base = g_world;
phiD_base_t0_p = zeros(3,1);    % HIER!
phi_base_t0_p = [pi/2; 0; 0];
r_base_t0_p =  [-0.1131;-0.2539; -0.2539];%T_W_C_e_init(1:3,4);          % HIER!
rD_base_t0_p = zeros(3,1);

%% kinematic parameters 
a4 = 0.363;
a5 = 0.0;
d4 = .149; % new
% d5 = 0.25686;
d5 = 0.29696; % new
t1 = [a4; a5; d4; d5;];
pkin_p = t1 ;

%% wrist parameters

% modify default settings
% addpath(fullfile(fileparts(this_path_), '00_lib'));

% run('wrist_para_B01.m')
% wp.T_wb_B1
% wp.T_wb_B2
% wp.T_wb_B3
% wp.r_4_4wb
% wp.r_E_E_T1
% wp.r_E_E_T2
% wp.r_E_E_T3
% 
% k_virtual = 300000;
% d_virtual = 450;     
% 
% r_E_E_T1 = wp.r_E_E_T1;
% r_E_E_T2 = wp.r_E_E_T2;
% r_E_E_T3 = wp.r_E_E_T3;
% r_4_4wb = wp.r_4_4wb;
% T_wb_B1 = wp.T_wb_B1;
% T_wb_B2 = wp.T_wb_B2;
% T_wb_B3 = wp.T_wb_B3;
% R1 = wp.R1;
% R2 = wp.R2;
% R_x_dir = wp.R_x_dir;
% d_offset_tendon = wp.d_offset_tendon;
% delta_rot = wp.delta_rot;
% h = wp.h;
% 
% ap.el.i_gear = 51;
% ap.el.k_m = 0.0369; % motor constant
% ap.el.i_gear_vec = [100 100 100 100 ap.el.i_gear ap.el.i_gear ap.el.i_gear ap.el.i_gear]; %overwrite
% ap.el.k_m_vec = [0.04 0.04 0.04 0.04 ap.el.k_m ap.el.k_m ap.el.k_m ap.el.k_m]; %overwrite
% 
% % ap.el.r_p = 0.01531;
% ap.el.r_p = 0.0035;



%% Mechanical parameters

s_mp_in = fct_param_boxes_B01_wr_desktop(); % paylod 1.
frame_index_order = {'T_0_0', 'T_0_sh1', 'T_0_sh2','T_0_sh3', 'T_0_el', 'T_0_fa', 'T_0_wj1', 'T_0_wj2'};

close all
f1 = figure;
s_mp_out = fct_calc_mass_parameter_box(s_mp_in);

T_c_mdh = exoprB01_fkine_fixb_rotmat_mdh_sym_varpar(qJ_t0_p, pkin_p);

R_W_0 = eul2rotm(phi_base_t0_p','XYZ');
T_W_0 = [R_W_0, r_base_t0_p; 0, 0, 0, 1];
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


% figure
fct_plot_boxes(s_plot_edges_out, s_mp_out)
  view(22.5,35.6)
  xlabel('x')
  ylabel('y')
  zlabel('z')
  grid on;
  axis equal
  
TF_.KSW.T_W_0 = T_W_0;
TF_.KSW.T_W_1_p = T_W_0*T_c_mdh(:,:,2);
TF_.KSW.T_W_2_p = T_W_0*T_c_mdh(:,:,3);
TF_.KSW.T_W_3_p = T_W_0*T_c_mdh(:,:,4);
TF_.KSW.T_W_4_p = T_W_0*T_c_mdh(:,:,5);
TF_.KSW.T_W_5_p = T_W_0*T_c_mdh(:,:,6);
TF_.KSW.T_W_6_p = T_W_0*T_c_mdh(:,:,7);
TF_.KSW.T_W_7_p = T_W_0*T_c_mdh(:,:,8);
  
% TF_.KSW = rmfield(TF_.KSW,'T_W_0');
TF_.KSW = rmfield(TF_.KSW,'T_W_1_p');
TF_.KSW = rmfield(TF_.KSW,'T_W_2_p');
TF_.KSW = rmfield(TF_.KSW,'T_W_3_p');
TF_.KSW = rmfield(TF_.KSW,'T_W_6_p');
TF_.KSW = rmfield(TF_.KSW,'T_W_7_p');

plot_KS_world(TF_, .1, .1)
fct_plot_KS_place_numbers_world(TF_)

if MODEL_PARAMETERS == 0 % box based or from solidworks model parameters
 [pkin_p_not_used, m_p, rSges_p, mrSges_p, Icges_p, Ifges_p] = ...
    param_proth_KinMech_ex2_box_sh(s_mp_out, frame_index_order);
else
    disp('reading data from SW')
    try
        [m_p, rSges_p, mrSges_p, Ifges_p, Icges_p] = read_SW_inertia(path___,  ...
            {'T_0_0', 'T_0_sh1', 'T_0_sh2','T_0_sh3', 'T_0_el', 'T_0_fa', 'T_0_wj1', 'T_0_wj2'});
    catch
        path___ = '/home/rtpc3/Documents';
        [m_p, rSges_p, mrSges_p, Ifges_p, Icges_p] = read_SW_inertia(path___,  ...
            {'T_0_0', 'T_0_sh1', 'T_0_sh2','T_0_sh3', 'T_0_el', 'T_0_fa', 'T_0_wj1', 'T_0_wj2'});
    end
end

path___calib = fullfile(filepath_, '00_lib','mechanical_parameters','02_calibweight');
[m_p_calib, rSges_p_calib, mrSges_p_calib, Ifges_p_calib, Icges_p_calib] = read_SW_inertia(path___calib,  ...
    {'T_0_0', 'T_0_sh1', 'T_0_sh2','T_0_sh3', 'T_0_el', 'T_0_fa', 'T_0_wj1', 'T_0_wj2'});

%%

%% Problem -> negative eigenwerte der Massenmatrix

if EXO_ONLY == 1
m_p = m_p.*[1 1 1 1 0 0 0 0]';
Ifges_p = Ifges_p.*[1 1 1 1 0 0 0 0]';
Icges_p = Icges_p.*[1 1 1 1 0 0 0 0]';
end

Ifges_p_ = Ifges_p;

MPVfloatb_p = exoprB01_convert_par2_MPV_floatb_eulangrpy(pkin_p, m_p, mrSges_p, Ifges_p_);
MPV_fixb_p = exoprB01_convert_par2_MPV_fixb(pkin_p, m_p, mrSges_p, Ifges_p_);
DynPar1_p = struct('m', m_p, 'r_S', rSges_p, 'I_S', Icges_p);
clear NQJ

% Mass matrix test
M_regmin = exoprB01_inertia_floatb_eulangrpy_regmin_slag_vp(zeros(NJ, 1), phi_base_t0_p, pkin_p);
M_ = exoprB01_inertia_floatb_eulangrpy_slag_vp1(zeros(NJ, 1), phi_base_t0_p, pkin_p, m_p, rSges_p, Icges_p);
Mq_small = exoprB01_inertiaJ_slag_vp1(zeros(NJ, 1), pkin_p, m_p, rSges_p, Icges_p);
if isreal(sqrtm(Mq_small)) == 0
    warning('FB Mass Matrix imaginary.')
end

M_mpv_vec = M_regmin*MPVfloatb_p;
Mq = vec2symmat(M_mpv_vec);
M1=sqrtm(Mq);
if isreal(M1) == 0
    error('Mass Matrix imaginary.')
end

%% Test initial tauJ

tauBg_reg = exoprB01_gravloadB_floatb_eulangrpy_regmin_slag_vp(qJ_t0_p, phi_base_t0_p, g_world, pkin_p);
tauJg_reg = exoprB01_gravloadJ_floatb_eulangrpy_regmin_slag_vp(qJ_t0_p, phi_base_t0_p, g_world, pkin_p);    
taug_reg = [tauBg_reg;tauJg_reg];
taug = taug_reg*MPVfloatb_p;
taug(7:end)

% Check if Coriolis and mass matrix provide large output for tiny input.
qJD = rand(7,1)*3e-3;


qJDD = rand(7,1)*3e-3;
xD_base = zeros(6,1);
tauc_reg = exoprB01_coriolisvec_floatb_eulangrpy_regmin_slag_vp(qJ_t0_p, qJD, ...
  phi_base_t0_p, xD_base, pkin_p);
tauc = tauc_reg*MPVfloatb_p;
M_regmin = exoprB01_inertia_floatb_eulangrpy_regmin_slag_vp(qJ_t0_p, phi_base_t0_p, pkin_p);
M_mpv_vec = M_regmin*MPVfloatb_p;
Mq = vec2symmat(M_mpv_vec);
tau_a = Mq(7:end, 7:end)*qJDD;
if any(tauc > 1) || any(tau_a > 1)
error('Coriolis and Mass matrix provide large output for tiny input.')
end


%% calculate param for prosthesis model 
a4__ = 0.33075;
a1__ = a4__/2;
d1__ = 0;
d2__ = 0.29696; % new
t1 = [a1__; d1__; d2__;];
pkin_po = t1 ;

df = 0.1;

m___ = zeros(5,1);
rSges___ = zeros(5,3);
Icges___ = zeros(5,6);

m___(1) = 0.1600;
rSges___(1,:) = [-0.0114         0   -0.0371];
Icges___(1,:) = [ 0.0001    0.0001    0.0002         0    0.0000         0];

m___(2:4,:) = m_p(end-3:end-1);
rSges___(2:4,:) = rSges_p(end-3:end-1,:);
Icges___(2:4,:) = Icges_p(end-3:end-1,:);


m___(end) = 0;
rSges___(end,:) = 0;
Icges___(end,:) = 0;

m_hand = [0 0 0 0 1]'*(0.385+0.08);
m_corr = [0 0 1 0 0]'*0.075;
r_hand = zeros(5,3);
r_hand(5,1) = 0.06;
I_hand = zeros(5,6);
I_hand(5,:) = [0.01 0.01 0.1 0 0 0];

m = m___ + m_hand + m_corr;
rSges = rSges___ + r_hand;
Icges = Icges___ + I_hand;

[mrSges, ... % first moment of all links (mass times center of mass)
    Ifges] = ... % second moment of all links (inertia around body frame origins)
    inertial_parameters_convert_par1_par2( rSges, Icges, m);

MPVfloatb = proth_wrist_no_ex2_convert_par2_MPV_floatb_eulangrpy(pkin_po, m, mrSges, Ifges);
DynPar1_po = struct('m', m, 'r_S', rSges, 'I_S', Icges);





