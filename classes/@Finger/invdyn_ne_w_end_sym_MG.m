% symbolic version:
% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% Fixed base
% 
% input:
%           sym_type: symbolic variables contains 
%               1: q,qd,qdd,f_ext
%               2: q,qd,qdd,f_ext,x_base,xd_base, xdd_base
%               3: q,qd,qdd,f_ext,x_base,xd_base, xdd_base, mass_all
% 
%               specific versions of definition
%               101: q,qd,qdd,f_ext,x_base,xd_base, xdd_base, mass_all(end)
%           save_res: Bin, save output as .m function 
% symbolic variables:
%           q_sym: [njx1]
%           qD_sym: [njx1]
%           qDD_sym: [njx1]
%           F_ext_ne_sym: [6x1]
% 
% symbolic output:
%           Tau_sym: [obj.njx1] saved in output/tau_sym_$obj.name$
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% TODO: need to adapt to the passive joint mode

function [M,FTau_G] = invdyn_ne_w_end_sym_MG(obj, sym_type, save_res)

% init symbolic terms (case 1)
q_sym = sym('q',[obj.nj,1], 'real');
qd_sym = sym(zeros(obj.nj,1));
% qdd_sym = sym('qdd',[obj.nj,1], 'real');
F_ext_sym = sym(zeros(6,1));

mdh_ne = mdh_struct_to_matrix(obj.mdh_ori, 1);
mdh_ne_sym = sym(mdh_ne);
Mass = obj.par_dyn_f.mass_all;
Mass_sym = sym(Mass);
X_base = zeros(6,1);
X_base(1:3) = obj.w_p_base;
X_base(4:6) = R2euler_XYZ(obj.w_R_base);
X_base_sym = sym(X_base);
XD_base = zeros(6,1);
XD_base_sym = sym(XD_base);
XDD_base = zeros(6,1);
XDD_base_sym = sym(XDD_base);

CoM_ne = obj.par_dyn_f.com_all;
CoM_ne_sym = sym(CoM_ne);
I_ne = obj.par_dyn_f.inertia_all;
I_ne_sym = sym(I_ne);
g = obj.par_dyn_f.g;
g_sym = sym(g);

% init symbolic variables based on sym_type
switch sym_type
    case 1
        var_name = {q_sym};
end
FTau_G = invdyn_lag_mdh_G_sym(q_sym, mdh_ne_sym, X_base_sym, Mass_sym, CoM_ne_sym, g_sym);
M = sym(zeros(obj.nj,obj.nj));
for i = 1:obj.nj
    qDD_i = zeros(obj.nj,1);
    qDD_i(i) = 1;
    qdd_sym = sym(qDD_i);
    [FTau_MG,~,~] = invdyn_ne_mdh_sym(q_sym,qd_sym,qdd_sym,mdh_ne_sym, Mass_sym,...
             X_base_sym, XD_base_sym, XDD_base_sym, F_ext_sym,...
             CoM_ne_sym, I_ne_sym, g_sym);
    FTau_M = simplify(FTau_MG - FTau_G);
    M(:,i) = FTau_M;
end


if save_res
    
    if ~exist('./output', 'dir')
       mkdir('./output');
    end
    func_name = strcat('output/M_', obj.name);
    matlabFunction(M,"File",func_name,...
        "Vars", var_name,"Outputs", {'M_Matrix'});
    func_name_G = strcat('output/G_', obj.name);
    matlabFunction(FTau_G,"File",func_name_G,...
        "Vars", var_name,"Outputs", {'G_torque'});
end

end