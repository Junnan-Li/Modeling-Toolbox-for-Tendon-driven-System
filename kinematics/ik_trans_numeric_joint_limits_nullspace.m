% numeric inverse kinematic methods: inverse Jacobian 
% 
% Outputs:
%           q
%           status: 0: no solution within tol; 
%                   1: ik problem solved without nullspace
%                   2: ik problem solved with nullspace
% 
% TODO: 
%   base rotation and position not considered



function [q,status,q_all,x_res,phi_x,iter,q_null_res,phi_x_null] = ik_trans_numeric_joint_limits_nullspace(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha,q_diff_min,mex)
%#codegen
coder.extrinsic('ik_trans_numeric_joint_limits_mex')

status = 0;
q_i = q_init;
n_q = length(q_i);


q = zeros(n_q,1);
q_null_i = zeros(n_q,1);
q_null_res = zeros(n_q,1);
delta_q = zeros(n_q,1);
q_all = zeros(iter_max,n_q);
x_res = zeros(3,1);
phi_x = ones(3,1);
x_i = zeros(3,1);
iter = 0;
phi_x_null = zeros(3,1);

if mex
    [q,status,q_all,x_res,phi_x,iter] = ik_trans_numeric_joint_limits_mex(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha);
else
    [q,status,q_all,x_res,phi_x,iter] = ik_trans_numeric_joint_limits(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha);
end
if status == 0 % no solution
%     q_null_res = 0;
    return
end

% nullspace
% phi_x_null_min = zeros(3,1);
for i = 1:100 
    q_init = rand(n_q,1).*(q_limits(:,2)-q_limits(:,1))+q_limits(:,1);
    if mex
        [q_null_i,status_null,~,~,phi_x_null_i,~] = ik_trans_numeric_joint_limits_mex(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha);
    else
        [q_null_i,status_null,~,~,phi_x_null_i,~] = ik_trans_numeric_joint_limits(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha);
    end
    delta_q = q_null_i - q;
%     q_null_res = q_null;
    if status_null && max(abs(delta_q)) > q_diff_min
        q_null_res = q_null_i;
        phi_x_null = phi_x_null_i;
        status = 11;
        break
    elseif status_null
        q_null_res = q_null_i;
        phi_x_null = phi_x_null_i;
    end

end
end


