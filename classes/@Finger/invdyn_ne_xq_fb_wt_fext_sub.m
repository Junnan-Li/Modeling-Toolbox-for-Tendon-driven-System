% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% floating base version 
% call invdyn_ne_xq_mdh_all_fext.m
% 
% Note: in this function, obj.w_p_base & obj.w_R_base are not passed to the
% calculation, need to give as input xq
% input:
%           xq: [6+obj.nj]
%           xqD: [6+obj.nj] 
%           xqDD: [6+obj.nj]
%           F_ext_ne_all_fext: [6x(obj.nj+2)]: [f_base;f_ext_joint;f_ee]
% 
% output:
%           FTau: [6+obj.nj] 
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% TODO: need to adapt to the passive joint mode

function [FTau_G,FTau_C,M_xq,FTau_M,FTau_wt_fext] = invdyn_ne_xq_fb_wt_fext_sub(obj, xq, xqD, xqDD)

assert(length(xq)== obj.nj+6, 'dimension of joint vector is incorrect!')
assert(length(xqD)== obj.nj+6, 'dimension of joint vector is incorrect!')
assert(length(xqDD)== obj.nj+6, 'dimension of joint vector is incorrect!')

mdh_ne = mdh_struct_to_matrix(obj.mdh_ori, 1);
mdh_ne(1:obj.nj,3) = mdh_ne(1:obj.nj,3);
Mass = obj.par_dyn_f.mass_all;

CoM_ne = obj.par_dyn_f.com_all;
I_ne = obj.par_dyn_f.inertia_all;
g = obj.par_dyn_f.g;


[FTau_G,FTau_C,M_xq,FTau_M,FTau_wt_fext] = invdyn_ne_xq_mdh_wt_fext_sub(xq,xqD,xqDD,mdh_ne,Mass,CoM_ne, I_ne, g);

end