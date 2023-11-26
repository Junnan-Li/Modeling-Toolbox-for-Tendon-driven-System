% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% 
% floating base version 
% symbolic version
% without external force
% calculate sub-terms
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


function [FTau_G,FTau_C,M_xq,FTau_M,FTau_wt_fext] = invdyn_ne_xq_mdh_wt_fext_sub_sym(xq,xqD,xqDD,mdh_ne,Mass,CoM_ne, I_ne, g)
%#codegen


n_state = length(xq);
n_q = n_state-6;
f_ext = sym(zeros(6,n_q+2));

% calculate FTau without exteranl force
[FTau_wt_fext,~] = invdyn_ne_xq_mdh_all_fext_sym(xq,xqD,xqDD,...
            mdh_ne,Mass,f_ext, CoM_ne, I_ne, g);

%% gravity term
% xqD = 0; xqDD = 0; F_ext = 0;
[FTau_G,~] = invdyn_ne_xq_mdh_all_fext_sym(xq,sym(zeros(n_state,1)),sym(zeros(n_state,1)),mdh_ne,Mass,f_ext, CoM_ne, I_ne, g);

%% Coriolis term
% xqD = 0; xqDD = 0; F_ext = 0;
FTau_CG = invdyn_ne_xq_mdh_all_fext_sym(xq,xqD,sym(zeros(n_state,1)),mdh_ne,Mass,f_ext, CoM_ne, I_ne, g);
FTau_C = simplify(FTau_CG - FTau_G);

%% Mass matrix
% xqD = 0, F_ext = 0
M_xq = zeros(n_state,n_state);
for i = 1:n_state
    xqDD_i = zeros(n_state,1);
    xqDD_i(i) = 1;
    xqDD_i_sym = sym(xqDD_i);
    [FTau_MG,~] = invdyn_ne_xq_mdh_all_fext_sym(xq,sym(zeros(n_state,1)),xqDD_i_sym,mdh_ne,Mass,f_ext, CoM_ne, I_ne, g);
    FTau_M = simplify(FTau_MG - FTau_G);
    M_xq(:,i) = FTau_M;
end
FTau_M = simplify(M_xq * xqDD);

% % for validating: other method of calculate FTau_M
% [FTau_MG,~] = invdyn_ne_xq_mdh_all_fext(xq,zeros(n_state,1),xqDD,mdh_ne,Mass,zeros(6,n_q+2), CoM_ne, I_ne, g);
% FTau_M = FTau_MG - FTau_G;

end