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

function [Tau_sym, Tau_G_sym] = invdyn_ne_w_end_sym(obj, sym_type, save_res)

% init symbolic terms (case 1)
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

% init symbolic variables based on sym_type
switch sym_type
    case 1
        var_name = [q_sym;qd_sym;qdd_sym;F_ext_sym];
    case 2
        X_base_sym = sym('x_base',[6,1], 'real');
        XD_base_sym = sym('x_base',[6,1], 'real');
        XDD_base_sym = sym('x_base',[6,1], 'real');
        var_name = [q_sym;qd_sym;qdd_sym;F_ext_sym;X_base_sym;XD_base_sym;XDD_base_sym];
    case 3
        X_base_sym = sym('x_base',[6,1], 'real');
        XD_base_sym = sym('x_base',[6,1], 'real');
        XDD_base_sym = sym('x_base',[6,1], 'real');
        Mass_sym = sym('mass', size(Mass), 'positive');
        var_name = [q_sym;qd_sym;qdd_sym;F_ext_sym;X_base_sym;XD_base_sym;XDD_base_sym;Mass_sym];
    case 101
        X_base_sym = sym('x_base',[6,1], 'real');
        XD_base_sym = sym('xd_base',[6,1], 'real');
        XDD_base_sym = sym('xdd_base',[6,1], 'real');
        mass_end = sym('m_end','positive');
        Mass_sym = sym([Mass(1:end-1);mass_end]);
        CoM_end = sym('com_end', [3,1],'real');
        CoM_ne_sym = sym([CoM_ne(:,1:end-1),CoM_end]);
        I_end = sym('I_end', [6,1],'real');
        I_ne_sym = sym([I_ne(:,1:end-1),I_end]);
        var_name = [q_sym;qd_sym;qdd_sym;F_ext_sym;X_base_sym;XD_base_sym;XDD_base_sym;mass_end;CoM_end;I_end];
end


[Tau_sym,~,~] = invdyn_ne_mdh_sym(q_sym,qd_sym,qdd_sym,mdh_ne_sym, Mass_sym,...
             X_base_sym, XD_base_sym, XDD_base_sym, F_ext_sym,...
             CoM_ne_sym, I_ne_sym, g_sym);

% gravity term
switch sym_type
    case 1
        Tau_G_sym  = simplify(subs(Tau_sym, [qd_sym;qdd_sym;F_ext_sym], ...
            [zeros(obj.nj,1);zeros(obj.nj,1);zeros(6,1)]));
    case 2
        Tau_G_sym  = simplify(subs(Tau_sym, [qd_sym;qdd_sym;XD_base_sym,XDD_base_sym,F_ext_sym], ...
            [zeros(obj.nj,1);zeros(obj.nj,1);zeros(6,1);zeros(6,1);zeros(6,1)]));
    case 3
        Tau_G_sym  = simplify(subs(Tau_sym, [qd_sym;qdd_sym;XD_base_sym,XDD_base_sym,F_ext_sym], ...
            [zeros(obj.nj,1),zeros(obj.nj,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1)]));
    case 101
        Tau_G_sym  = simplify(subs(Tau_sym, [qd_sym;qdd_sym;XD_base_sym,XDD_base_sym,F_ext_sym], ...
            [zeros(obj.nj,1),zeros(obj.nj,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1)]));
end
% Mass term
% qd = 0, F_ext = 0
% M = zeros(obj.nj,obj.nj)
% for i = 1:obj.nj
%     xqDD_i = zeros(n_state,1);
%     xqDD_i(i) = 1;
%     simplify(subs(Tau_sym, [qd_sym,qdd_sym,XD_base_sym,XDD_base_sym,F_ext_sym], ...
%                 [zeros(obj.nj,1),zeros(obj.nj,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1)]));
%     M_xq(:,i) = FTau_CG - G_xq;
% end


if save_res
    func_name = strcat('output/tau_sym_', obj.name);
    if ~exist('./output', 'dir')
       mkdir('./output');
    end
    matlabFunction(Tau_sym,"File",func_name,...
        "Vars", var_name,"Outputs", {'Tau'});
end

end