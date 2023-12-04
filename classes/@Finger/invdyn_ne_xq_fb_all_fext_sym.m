% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% floating base version 
% symbolic version
% call invdyn_ne_xq_mdh_all_fext_sym.m
% 
% Note: in this function, obj.w_p_base & obj.w_R_base are not passed to the
% calculation, need to give as input xq
% input:
%           sym_type: symbolic variables contains 
%               1: xq,xqd,xqdd
%               2: xq,xqd,xqdd,f_ext
% 
%               specific versions of definition
%               101(christ): xq,xqd,xqdd,
%                            mass_all(end),com(end),inertia(end)
%           save_res: Bin, save output as .m function 
% 
% output:
%           FTau: [6+obj.nj] 
% 
% 
% Junnan Li, 11.2023

function [FTau_sym] = invdyn_ne_xq_fb_all_fext_sym(obj, sym_type, save_res)

% assert(length(xq)== obj.nj+6, 'dimension of joint vector is incorrect!')
% assert(length(xqD)== obj.nj+6, 'dimension of joint vector is incorrect!')
% assert(length(xqDD)== obj.nj+6, 'dimension of joint vector is incorrect!')

% init symbolic terms (case 1)
xq_sym = sym('xq',[obj.nj+6,1], 'real');
xqd_sym = sym('xqd',[obj.nj+6,1], 'real');
xqdd_sym = sym('xqdd',[obj.nj+6,1], 'real');
F_ext_all_sym = sym(zeros(6,obj.nj+2)); % default no external force variables


mdh_ne = mdh_struct_to_matrix(obj.mdh_ori, 1);
mdh_ne(1:obj.nj,3) = mdh_ne(1:obj.nj,3);
mdh_ne_sym = sym(mdh_ne);
Mass = obj.par_dyn_f.mass_all;
Mass_sym = sym(Mass);

CoM_ne = obj.par_dyn_f.com_all;
CoM_ne_sym = sym(CoM_ne);
I_ne = obj.par_dyn_f.inertia_all;
I_ne_sym = sym(I_ne);
g = obj.par_dyn_f.g;
g_sym = sym(g);


switch sym_type
    case 1
        var_name = {xq_sym,xqd_sym,xqdd_sym};
    case 2
        F_ext_sym = sym('F_ext_ee',[6,1], 'real');
        F_ext_all_sym = [sym(zeros(6,obj.nj+1)),F_ext_sym];
        var_name = {xq_sym,xqd_sym,xqdd_sym,F_ext_all_sym};
    case 101 % 
        mass_end = sym('m_end','positive');
        Mass_sym = sym([Mass(1:end-1);mass_end]);
        CoM_end = sym('com_end', [3,1],'real');
        CoM_ne_sym = sym([CoM_ne(:,1:end-1),CoM_end]);
        I_end = sym('I_end', [6,1],'real');
        I_ne_sym = sym([I_ne(:,1:end-1),I_end]);
        var_name = {xq_sym,xqd_sym,xqdd_sym,mass_end,CoM_end,I_end};
end


[FTau_sym,~] = invdyn_ne_xq_mdh_all_fext_sym(xq_sym,xqd_sym,xqdd_sym,...
            mdh_ne_sym,Mass_sym,F_ext_all_sym, CoM_ne_sym, I_ne_sym, g_sym);

if save_res
    func_name = strcat('output/tau_xq_fb_sym_', obj.name);
    if ~exist('./output', 'dir')
       mkdir('./output');
    end
    matlabFunction(FTau_sym,"File",func_name,...
        "Vars", var_name);
    save('./output',FTau_sym)
end

end