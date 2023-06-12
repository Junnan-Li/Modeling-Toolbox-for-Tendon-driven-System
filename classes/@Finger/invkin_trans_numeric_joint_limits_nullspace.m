% Numeric nullspace inverse kinematic method with joint limits
% 
% input:
%           x_des: desired position in world frame
%           iter_max: maximal number of iteration
%           tol: tolerance of the phi_x
%           alpha: change rate
% 
% output:
%           q: calculated joint angle
%           x_res: result of the position x
%           phi_x: PHI
%           iter: interation number 
% 
% Junnan Li, junnan.li@tum.de, 04.2023

function [q_res, status, q_all, x_res,phi_x,iter,q_n,phi_x_n] = invkin_trans_numeric_joint_limits_nullspace(obj,x_des,iter_max,tol,alpha,q_diff_min, mex)

if nargin == 2
    iter_max = 100;
    tol = 1e-9;
    alpha = 0.8;
    q_diff_min = 0.2;
    mex = 0;
elseif nargin == 3
    tol = 1e-9;
    alpha = 0.8;
    q_diff_min = 0.2;
    mex = 0;
elseif nargin == 4
    alpha = 0.8;
    q_diff_min = 0.2;
    mex = 0;
elseif nargin == 5
    q_diff_min = 0.2;
    mex = 0;
elseif nargin == 6
    mex = 0;
end
% assert(length(q)== obj.nja, 'dimension of joint vector is incorrect!')

q_all = zeros(iter_max,obj.nja);
q_res = zeros(size(obj.nja));
status = 0;
q_n = zeros(size(obj.nja));
phi_x_n = zeros(3,1);

q_limits = obj.limits_q(:,1:2);
% for j = 1:obj.nja
%     a(j) = (q_limits(j,2)+q_limits(j,1))/2;  
% end

w_p_base = obj.w_p_base;
w_R_base = obj.w_R_base;
% w_T_base = [obj.w_R_base,obj.w_p_base;0,0,0,1];

q_init = obj.q_a;
mdh_matrix = mdh_struct_to_matrix(obj.mdh_ori, 1);
x_des_mod = w_R_base' * (x_des - w_p_base);

if mex
    [q_mex,status_mex,q_all_mex,x_mex,phi_mex,iter_mex,q_null_mex,phi_x_null_mex] = ik_trans_numeric_joint_limits_nullspace_mex(mdh_matrix,x_des_mod,q_init,...
                                                     q_limits,iter_max,tol,alpha,q_diff_min,1);
    
    obj.update_finger(q_mex);
    q_res = q_mex;
    status = status_mex;
    q_all = q_all_mex;
    x_res = w_R_base * x_mex + w_p_base;
    phi_x = w_R_base * phi_mex;
    iter = iter_mex;
    q_n = q_null_mex;
    phi_x_n = phi_x_null_mex;
else

    [q_null,status_null,q_all_null,x_res_null,phi_x_null,iter_null,q_null_res,phi_x_null_2] = ik_trans_numeric_joint_limits_nullspace...
           (mdh_matrix,x_des_mod,q_init,q_limits,iter_max,tol,alpha,q_diff_min,0);
    obj.update_finger(q_null);
    q_res = q_null;
    status = status_null;
    q_all = q_all_null;
    x_res = w_R_base * x_res_null + w_p_base;
    phi_x = w_R_base * phi_x_null;
    iter = iter_null;
    q_n = q_null_res;
    phi_x_n = phi_x_null_2;

end

end

