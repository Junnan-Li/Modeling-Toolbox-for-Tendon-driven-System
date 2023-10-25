% Script of generating symbolic calculation sode for dynamics using Matlab
% Symbolic Toolbox





clear all
close all 
clc

%% load example model

run examples\create_planar_2dof.m

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

model = copy(robot_2dof);
nj = model.nj;
model.set_base(zeros(3,1), eye(3))
model.update_finger(zeros(2,1))

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
%%

[Tau,~,~] = invdyn_ne_mdh_sym(q_sym,qd_sym,qdd_sym,mdh_sym, Mass_sym,...
             sym(zeros(6,1)), sym(zeros(6,1)), sym(zeros(6,1)), F_ext_ne, CoM_ne_sym, I_ne_sym, g_sym);
Tau = simplify(Tau);
%%
q_0 = rand(2,1);
qd_0 = rand(2,1);
qdd_0 = rand(2,1);
Tau_num = model.invdyn_ne_w_end(q_0, qd_0, qdd_0, zeros(6,1))

Tau_sym = double(subs(Tau,[q_sym qd_sym qdd_sym], [q_0 qd_0 qdd_0]))