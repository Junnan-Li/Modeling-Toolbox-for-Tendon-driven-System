% Alexander Toedtheide Feb 2020

% from solidworks 
wp.R1 = 0.025; % m
wp.R_x_dir = wp.R1; % m
wp.R2 = 0.0075; % m
% akutell 0.2 = 0.14 + 0.06
wp.h = 0.04574; % m
wp.r_4_4wb = [0; 0; -wp.h];
wp.r_wb_wb_E = [0; 0; wp.h; 1];

% wp.r_B_B_C = [0; 0; rSges_p(5,1)];



Mode_torque_force = 0;
wp.F_pre_tension = 12.5;
% g_base = [0; 0; -9.81];
% g_world = g_base;
% T_ctrl = 5e-4;

% try
% omega_abc = (1200/2/pi)*2*pi;
% [num_abc_z_alpha, den_abc_z_alpha] = tf_filter_1st_order(omega_abc, T_ctrl, 1);
% omega_abc = (1200/2/pi)*2*pi;
% [num_abc_z_beta, den_abc_z_beta] = tf_filter_1st_order(omega_abc, T_ctrl, 1);
% end


wp.angle1 =  0;
wp.angle2 =  2/3*pi;
wp.angle3 = -2/3*pi;
wp.d_offset_tendon = 0.0;

wp.r_E_E_T1 =  [wp.R_x_dir*cos(wp.angle1); ...
    wp.R_x_dir*sin(wp.angle1); ...
    wp.d_offset_tendon];

wp.r_E_E_T2 = [wp.R1*cos(wp.angle2); ...
    wp.R1*sin(wp.angle2); ...
    wp.d_offset_tendon];

wp.r_E_E_T3 = [wp.R1*cos(wp.angle3); ...
    wp.R1*sin(wp.angle3); ...
    wp.d_offset_tendon];
r.KS0.r0KSE = [0; ...
    0; ...
    wp.h];
%


a = wp.R1*3/sqrt(3);         % m length of triangle

wp.r_wbB1 = wp.R2*[cos(wp.angle1); ...
    sin(wp.angle1)*wp.R2; ...
    0];

wp.r_wbB2 = wp.R2*[cos(wp.angle2); ...
    sin(wp.angle2); ...
    0];

wp.r_wbB3 = wp.R2*[cos(wp.angle3); ...
    sin(wp.angle3); ... KS0 platform mountpoint
    0];

wp.delta_rot = -pi/4;
T_wb_B1_fix = [rotz(wp.angle1), wp.r_wbB1; 0 0 0 1];
T_wb_B2_fix = [rotz(wp.angle2), wp.r_wbB2; 0 0 0 1];
T_wb_B3_fix = [rotz(wp.angle3), wp.r_wbB3; 0 0 0 1];

T_rot_ks1 = [roty(wp.delta_rot), [0; 0; 0]; 0 0 0 1];
T_rot_ks2 = [rotz(wp.delta_rot), [0; 0; 0]; 0 0 0 1];

wp.T_wb_B1 = T_wb_B1_fix*T_rot_ks1*T_rot_ks2;
wp.T_wb_B2 = T_wb_B2_fix*T_rot_ks1*T_rot_ks2;
wp.T_wb_B3 = T_wb_B3_fix*T_rot_ks1*T_rot_ks2;

qJ_t0_p = zeros(7,1);
warning('unclear if qJ needs to be exact')
T_wb_E_t0 = eul2rotm([0,-qJ_t0_p(end-1),-qJ_t0_p(end)], 'ZYX'); % hier aufpassen mit Vorzeichen!!!!
R_wb_E_t0 = T_wb_E_t0(1:3,1:3);

wp.r_wb_wb_T1 = [R_wb_E_t0*wp.r_E_E_T1 + wp.r_wb_wb_E(1:3); 1];
r_B1_B1_T1 = inv(wp.T_wb_B1)*wp.r_wb_wb_T1;
l1_0 = norm(r_B1_B1_T1(1:3), 2);

wp.r_wb_wb_T2 = [R_wb_E_t0*wp.r_E_E_T2 + wp.r_wb_wb_E(1:3); 1];
r_B2_B2_T2 = inv(wp.T_wb_B2)*wp.r_wb_wb_T2;
l2_0 = norm(r_B2_B2_T2(1:3), 2);

wp.r_wb_wb_T3 = [R_wb_E_t0*wp.r_E_E_T3 + wp.r_wb_wb_E(1:3); 1];
r_B3_B3_T3 = inv(wp.T_wb_B3)*wp.r_wb_wb_T3;
l3_0 = norm(r_B3_B3_T3(1:3), 2);

t_01_wait = 0.5;
t_02_fctrl_interp = 0.5;
t_03_fctrl_wait = 0.5;
t_00_start_trajectory = 5;
t_00_loose_wrist = 4.5;

% r_pulley = 0.01531;
% r_p = r_pulley;
% i_gear = 51;
