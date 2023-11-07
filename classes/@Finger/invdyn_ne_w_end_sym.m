% symbolic version:
% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% 
% input:
%           save_res: Bin, save output as .m function 
% symbolic variables:
%           q_sym: [njx1]
%           qD_sym: [njx1]
%           qDD_sym: [njx1]
%           F_ext_ne_sym: [6x1]
% 
% symbolic output:
%           Tau_sym: [obj.njx1] 
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% TODO: need to adapt to the passive joint mode

function Tau_sym = invdyn_ne_w_end_sym(obj, save_res)

q_sym = sym('q',[obj.nj,1], 'real');
qd_sym = sym('qd',[obj.nj,1], 'real');
qdd_sym = sym('qdd',[obj.nj,1], 'real');
F_ext_sym = sym('F_ext',[6,1], 'real');

mdh_ne = mdh_struct_to_matrix(obj.mdh_ori, 1);
mdh_ne(1:obj.nj,3) = mdh_ne(1:obj.nj,3);
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

[Tau_sym,~,~] = invdyn_ne_mdh_sym(q_sym,qd_sym,qdd_sym,mdh_ne_sym, Mass_sym,...
             X_base_sym, XD_base_sym, XDD_base_sym, F_ext_sym,...
             CoM_ne_sym, I_ne_sym, g_sym);

if save_res
    func_name = strcat('output/tau_sym_', obj.name);
    if ~exist('./output', 'dir')
       mkdir('./output');
    end
    matlabFunction(Tau_sym,"File",func_name,...
        "Vars", {q_sym,qd_sym,qdd_sym,F_ext_sym});
end

end