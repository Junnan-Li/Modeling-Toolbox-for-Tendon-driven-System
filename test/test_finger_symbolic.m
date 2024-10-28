% test symbolic representation of Finger dynamics
% Finger.invdyn_ne_w_end_sym



clear all
close all
clc

%%%%%


finger_3dof = create_finger_random('finger_example', 3);
% set random states
% set random base position and orientation
finger_3dof.set_base(4*rand(3,1),euler2R_XYZ(rand(1,3)));
% finger_3dof.w_R_base = euler2R_XYZ(rand(1,3));

% % generate the symbolic term
tic
Tau_sym = finger_3dof.invdyn_ne_w_end_sym(1,1);
t = toc;
fprintf('Time cost of szmbolic generation: %d \n', t)
% [FTau_sym] = finger_3dof.invdyn_ne_xq_fb_all_fext_sym(1, 1);


%% validate symbolic representation
q_rand = rand(finger_3dof.nj,1);
qd_rand = rand(finger_3dof.nj,1);
qdd_rand = rand(finger_3dof.nj,1);

F_ext_rand  = rand(6,1);

Tau_num = finger_3dof.invdyn_ne_w_end(q_rand, qd_rand , qdd_rand , F_ext_rand );

q_sym = sym('q',[2,1], 'real');
qd_sym = sym('qd',[2,1], 'real');
qdd_sym = sym('qdd',[2,1], 'real');
F_ext_sym = sym('F_ext',[6,1], 'real');
% Tau_sym_res = double(subs(Tau_sym,symvar(Tau_sym), [F_ext_rand ;q_rand ;qd_rand ;qdd_rand]')); % [q_sym qd_sym qdd_sym F_ext_sym],
Tau_sym_res = tau_sym_finger_example(q_rand, qd_rand , qdd_rand , F_ext_rand );
Tau_error = Tau_num-Tau_sym_res;

if max(abs(Tau_error(:))) > 1e-10 
    test_12_status = 0;
    fprintf('Test (sym torque): failed! \n')
else
    fprintf('Test (sym torque): pass! \n')
end
%% floating base test
% generate symbolic function

% [FTau_sym] = finger_3dof.invdyn_ne_xq_fb_all_fext_sym(1, 1);
% [FTau_G,FTau_C,M_xq,FTau_M,FTau_wt_fext] = finger_3dof.invdyn_ne_xq_fb_wt_fext_sub_sym_par(1,1);

%
xq = rand(6+finger_3dof.nj,1);
xqd = rand(6+finger_3dof.nj,1);
xqdd = rand(6+finger_3dof.nj,1);

tic 
Tau_fb_num = finger_3dof.invdyn_ne_xq_fb_all_fext(xq, xqd, xqdd, zeros(6,2+finger_3dof.nj));
t1 = toc;
% tic
% Tau_fb_sym_res = double(subs(FTau_wt_fext,symvar(FTau_wt_fext), [xq(4:end) ;xqd(4:end) ;xqdd]')); 
% t2 = toc;
tic
FTau_fb_sym = tau_xq_fb_sym_finger_3dof_FTau_wt_fext(xq,xqd,xqdd);
t3 = toc;
Tau_fb_error = Tau_fb_num-Tau_fb_sym_res;
Tau_fb_error2 = Tau_fb_num - FTau_fb_sym;

fprintf('Test 12 num t: %f! \n',t1)
fprintf('Test 12 sym sub t: %f! \n',t2)
fprintf('Test 12 sym t: %f! \n',t3)

if max(abs(Tau_fb_error2(:))) > 1e-10 | max(abs(Tau_fb_error(:))) > 1e-10
    test_12_status = 0;
    fprintf('Test 12 (sym torque): failed! \n')
else
    fprintf('Test 12 (sym torque): pass! \n')
end

%% test 13 the sub terms of the fb dynamics 
% symbolic vs. fordyn_ne_w_end
finger_3dof.set_base(xq(1:3),euler2R_XYZ(xq(4:6)));

% test mass coriolis and gravity
[~,M_fd_1,C_fd_1,G_fd_1] = finger_3dof.fordyn_ne_w_end(xq(7:end),xqd(7:end),zeros(2,1), zeros(6,4),0);

FTau_fb_sym_M = tau_xq_fb_sym_finger_3dof_M(xq,xqd,xqdd);
FTau_fb_sym_m = tau_xq_fb_sym_finger_3dof_FTau_M(xq,xqd,xqdd);
FTau_fb_sym_G = tau_xq_fb_sym_finger_3dof_FTau_G(xq,xqd,xqdd);
FTau_fb_sym_C = tau_xq_fb_sym_finger_3dof_FTau_C(xq,xqd,xqdd);
FTau_fb_sym_C_0 = tau_xq_fb_sym_finger_3dof_FTau_C(xq,[zeros(6,1);xqd(7:end)],xqdd);
Tau_fb_all_error = FTau_fb_sym - FTau_fb_sym_m - FTau_fb_sym_G - FTau_fb_sym_C;

Tau_fb_M_error = M_fd_1 - FTau_fb_sym_M(7:end,7:end);
Tau_fb_G_error = G_fd_1 - FTau_fb_sym_G(7:end);
Tau_fb_C_error = C_fd_1 - FTau_fb_sym_C_0(7:end);


%% test 14 Lagrange methods

% generate mdh and Finger model

% use mdh to create finger
mdh_parameter = rand(4,4);
mdh_parameter(:,3) = 0;
mdh_parameter(1,1:4) = 0;
mdh_struct = mdh_matrix_to_struct(mdh_parameter, 1);
finger_r = Finger('3dof', 'mdh',mdh_struct );

% set random base position and orientation
x_base = rand(6,1);
finger_r.set_base(x_base(1:3),euler2R_XYZ(x_base(4:6)));

% finger_r.update_rst_model;
% joint configurations


n_q = finger_r.nj;

q_r = rand(n_q,1);
q_rd = rand(n_q,1);
q_rdd = rand(n_q,1);
% udpate finger with given joint configurations
finger_r.update_finger(q_r);
finger_r.plot_finger;


[qDD,M_fd,C_fd,G_fd] = finger_r.fordyn_ne_w_end(q_r, q_rd, zeros(n_q,1), zeros(6,n_q+2),0);

% Lagranian numeric
M_num = invdyn_lag_mdh_M(q_r, mdh_parameter, x_base, finger_r.par_dyn_f.mass_all, finger_r.par_dyn_f.com_all, finger_r.par_dyn_f.inertia_all);
G_num = invdyn_lag_mdh_G(q_r, mdh_parameter, x_base, finger_r.par_dyn_f.mass_all, finger_r.par_dyn_f.com_all, finger_r.par_dyn_f.g);

% Lagranian symbolic 
[Tau, M, C, G] = invdyn_lag_mdh_sym(sym(q_r),sym(q_rd),sym(q_rdd),sym(mdh_parameter), x_base,...
                sym(finger_r.par_dyn_f.mass_all), sym(finger_r.par_dyn_f.com_all), sym(finger_r.par_dyn_f.inertia_all), sym(finger_r.par_dyn_f.g));

M_lag_error = [M_fd-M_num;M_fd-M];
G_lag_error = vpa([G_fd-G_num;G_fd-G],7);
C_lag_error = vpa(C_fd - C*q_rd,7);

if max(abs(M_lag_error(:))) > 1e-10 |  max(abs(G_lag_error(:))) > 1e-10 | max(abs(C_lag_error(:))) > 1e-10 
    test_12_status = 0;
    fprintf('Test 14 (Lagrangian euler): failed! \n')
else
    fprintf('Test 14 (Lagrangian euler): pass! \n')
end

%% Test 15: Lagrange Euler sub symbolic method 

% generate mdh and Finger model

% use mdh to create finger
mdh_parameter = rand(4,4);
mdh_parameter(:,3) = 0;
mdh_parameter(1,1:4) = 0;
mdh_struct = mdh_matrix_to_struct(mdh_parameter, 1);
finger_r = Finger('Index', 'mdh',mdh_struct );

% set random base position and orientation
x_base = rand(6,1);
finger_r.w_p_base = x_base(1:3);
finger_r.w_R_base = euler2R_XYZ(x_base(4:6));
% finger_r.update_rst_model;
% joint configurations
n_q = finger_r.nj;

q_r = rand(n_q,1);
q_rd = rand(n_q,1);
q_rdd = rand(n_q,1);
% udpate finger with given joint configurations
finger_r.update_finger(q_r);
finger_r.plot_finger;

% symbolic variables
q_r_sym = sym('q',[n_q,1], 'real');
q_rd_sym = sym(q_rd);


% M = invdyn_lag_mdh_M_sym(q_r_sym, sym(mdh_parameter),sym(x_base), sym(finger_r.par_dyn_f.mass_all) , sym(finger_r.par_dyn_f.com_all), sym(finger_r.par_dyn_f.inertia_all))
% G = invdyn_lag_mdh_G_sym(q_r_sym, sym(mdh_parameter),sym(x_base), sym(finger_r.par_dyn_f.mass_all) , sym(finger_r.par_dyn_f.com_all), sym(finger_r.par_dyn_f.g))
[Tau, M, C, G] = invdyn_lag_mdh_sym(q_r_sym,q_rd_sym,sym(zeros(n_q,1)), sym(mdh_parameter),sym(x_base), sym(finger_r.par_dyn_f.mass_all),...
                        sym(finger_r.par_dyn_f.com_all), sym(finger_r.par_dyn_f.inertia_all), sym(finger_r.par_dyn_f.g)); %

func_name = strcat('output/invdyn_lag_mdh_sym_', finger_r.name);
matlabFunction(M,"File",strcat(func_name,'_FTau_G'),...
        "Vars", var_name);

M_sym = vpa(subs(M,q_r_sym,q_r),5);
G_sym = vpa(subs(G,q_r_sym,q_r),5);
[qDD,M_fd,C_fd,G_fd] = finger_r.fordyn_ne_w_end(q_r, q_rd, zeros(n_q,1), zeros(6,n_q+2),0);

M_lag_error = M_fd-M_sym;
G_lag_error = G_fd-G_sym;


C_sym = vpa(subs(C,q_r_sym,q_r),4);
C_sym*q_rd
