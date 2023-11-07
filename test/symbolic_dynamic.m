% Script of generating symbolic calculation sode for dynamics using Matlab
% Symbolic Toolbox





clear all
close all 
clc

%% load example model

run examples\create_planar_2dof.m
% run examples\christ\create_model.m

Tau_sym = robot_2dof.invdyn_ne_w_end_sym(1)

% run examples\create_finger.m
%% invdyn
% symbolic terms:
%   [q_1, ..., q_n]
%   [qd_1, ..., qd_n]
%   [qdd_1, ..., qdd_n]
%   [dyn_m_1, ..., dyn_m_n]
%   [dyn_com_1, ..., dyn_com_n]
%   [dyn_i_1, ..., dyn_i_n]
%   mdh
%   g
%   [x_1, ..., x_n]
%   [xd_1, ..., xd_n]
%   [xdd_1, ..., xdd_n]

model = copy(prost_low);
nj = model.nj;
model.set_base(zeros(3,1), eye(3))
model.update_finger(zeros(nj,1))

mdh_ne = mdh_struct_to_matrix(model.mdh_ori, 1);
mdh_ne(1:nj,3) = mdh_ne(1:nj,3);
Mass = model.par_dyn_f.mass_all;
% X_base = zeros(6,1);
% X_base(1:3) = model.w_p_base;
% X_base(4:6) = R2euler_XYZ(model.w_R_base);
% XD_base = zeros(6,1);
% XDD_base = zeros(6,1);
% F_ext_ne = F_ext;
CoM_ne = model.par_dyn_f.com_all;
I_ne = model.par_dyn_f.inertia_all;
g = model.par_dyn_f.g;

%%



q_sym = sym('q',[nj,1], 'real');
qd_sym = sym('qd',[nj,1], 'real');
qdd_sym = sym('qdd',[nj,1], 'real');
mdh_sym = sym(mdh_ne);
Mass_sym = sym(Mass);
F_ext_ne = sym(zeros(6,1));
CoM_ne_sym = sym(model.par_dyn_f.com_all);
I_ne_sym = sym(model.par_dyn_f.inertia_all);
g_sym = sym(model.par_dyn_f.g);

% return
%% symbolic calculation

[Tau,~,~] = invdyn_ne_mdh_sym(q_sym,qd_sym,qdd_sym,mdh_sym, Mass_sym,...
             sym(zeros(6,1)), sym(zeros(6,1)), sym(zeros(6,1)), F_ext_ne, CoM_ne_sym, I_ne_sym, g_sym);
Tau = simplify(Tau);
%% Validation
q_init = rand(nj,1);
model.update_finger(q_init);
q_0 = pi*rand(nj,1);
qd_0 = pi*rand(nj,1);
qdd_0 = pi*rand(nj,1);
tic
Tau_num = model.invdyn_ne_w_end(q_0, qd_0, qdd_0, zeros(6,1))
t1 = toc;
model.update_finger(q_init);
tic
Tau_sym = double(subs(Tau,[q_sym qd_sym qdd_sym], [q_0 qd_0 qdd_0]))
t2 = toc;
tic  
Tau_sym_gen = tau_prost_low(q_0,qd_0,qdd_0)
t3 = toc;
fprintf('Time cost of invdyn computation: \n')
fprintf('class function: %.4f \n', t1)
fprintf('symbolic function direct: %.4f \n', t2)
fprintf('symbolic function generated: %.4f \n', t3)
%% Code generation

% matlabFunction(Tau,"File","tau_prost_low","Vars", {q_sym,qd_sym,qdd_sym});


