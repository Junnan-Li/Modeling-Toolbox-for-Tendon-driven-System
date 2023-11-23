% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% 
% floating base version 
% symbolic version
% calculate sub-terms
% call invdyn_ne_xq_mdh_all_fext_sub_sym.m
% 
% Note: in this function, obj.w_p_base & obj.w_R_base are not passed to the
% calculation, need to give as input xq
% input:
%           sym_type: symbolic variables contains 
%               1: xq,xqd,xqdd
% 
%               specific versions of definition
%               101(christ): xq,xqd,xqdd,
%                            mass_all(end),com(end),inertia(end)
%           save_res: Bin, save output as .m function 
% 
% output:
%           FTau_G: [6+obj.nj] 
%           FTau_C: [6+obj.nj] 
%           M_xq: [6+obj.nj x 6+obj.nj] 
%           FTau_M: [6+obj.nj] 
%           FTau_wt_fext: [6+obj.nj] 
% 
% 
% 
% Junnan Li, 11.2023

function [FTau_G,FTau_C,M_xq,FTau_M,FTau_wt_fext] = invdyn_ne_xq_fb_wt_fext_sub_sym(obj, sym_type, save_res)

% assert(length(xq)== obj.nj+6, 'dimension of joint vector is incorrect!')
% assert(length(xqD)== obj.nj+6, 'dimension of joint vector is incorrect!')
% assert(length(xqDD)== obj.nj+6, 'dimension of joint vector is incorrect!')

% init symbolic terms (case 1)
xq_sym = sym('xq',[obj.nj+6,1], 'real');
xqd_sym = sym('xqd',[obj.nj+6,1], 'real');
xqdd_sym = sym('xqdd',[obj.nj+6,1], 'real');

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
    case 101 % 
        mass_end = sym('m_end','positive');
        Mass_sym = sym([Mass(1:end-1);mass_end]);
        CoM_end = sym('com_end', [3,1],'real');
        CoM_ne_sym = sym([CoM_ne(:,1:end-1),CoM_end]);
        I_end = sym('I_end', [6,1],'real');
        I_ne_sym = sym([I_ne(:,1:end-1),I_end]);
        var_name = {xq_sym,xqd_sym,xqdd_sym,mass_end,CoM_end,I_end};
end


[FTau_G,FTau_C,M_xq,FTau_M,FTau_wt_fext] = invdyn_ne_xq_mdh_wt_fext_sub_sym(xq_sym,xqd_sym,xqdd_sym,...
            mdh_ne_sym,Mass_sym, CoM_ne_sym, I_ne_sym, g_sym);

if save_res
    func_name = strcat('output/tau_xq_fb_sym_', obj.name);
    if ~exist('./output', 'dir')
       mkdir('./output');
    end
    matlabFunction(FTau_sym,"File",strcat(func_name,'_FTau_G'),...
        "Vars", var_name);
    matlabFunction(FTau_C,"File",strcat(func_name,'_FTau_C'),...
        "Vars", var_name);
    matlabFunction(M_xq,"File",strcat(func_name,'_M'),...
        "Vars", var_name);
    matlabFunction(FTau_M,"File",strcat(func_name,'_FTau_M'),...
        "Vars", var_name);
    matlabFunction(FTau_wt_fext,"File",strcat(func_name,'_FTau_wt_fext'),...
        "Vars", var_name);
end

end