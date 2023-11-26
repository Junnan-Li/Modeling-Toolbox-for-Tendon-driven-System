% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% 
% floating base version 
% symbolic version
% without external force
% calculate sub-terms
% leveraging parallel computation
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
%           FTau_G: [6+obj.nj] 
%           FTau_C: [6+obj.nj] 
%           M_xq: [6+obj.nj x 6+obj.nj] 
%           FTau_M: [6+obj.nj] 
%           FTau_wt_fext: [6+obj.nj] 
% 


function [FTau_G,FTau_C,M_xq,FTau_M,FTau_wt_fext] = invdyn_ne_xq_mdh_wt_fext_sub_sym_par(xq,xqD,xqDD,mdh_ne,Mass,CoM_ne, I_ne, g)
%#codegen


n_state = length(xq);
n_q = n_state-6;
f_ext = sym(zeros(6,n_q+2));

% use parfor loop to compute 
% dimension: [n_state x n_state+3]
%   1. FTau_wt_fext: normal invdyn_ne_xq_mdh_all_fext_sym.m wt external force
%   2. FTau_G: with xq only
%   3. FTau_C: with xq,xqd only


xq_loop = repmat(xq,1,3+n_state);
xqd_loop = [xqD,sym(zeros(n_state,1)),xqD,sym(zeros(n_state,n_state))];
xqdd_loop = [xqDD,sym(zeros(n_state,2)),sym(eye(n_state))];

FTau_par_all = sym([]);

parfor i = 1:n_state+3
    FTau_par_all(:,i) = invdyn_ne_xq_mdh_all_fext_sym(xq_loop(:,i),xqd_loop(:,i),xqdd_loop(:,i),...
            mdh_ne,Mass,f_ext, CoM_ne, I_ne, g);

end
FTau_wt_fext = simplify(FTau_par_all(:,1));
FTau_G = simplify(FTau_par_all(:,2));
FTau_C = simplify(FTau_par_all(:,3) - FTau_G);

M_xq = simplify(FTau_par_all(:,4:end)-FTau_G);
FTau_M = simplify(M_xq * xqDD);

end