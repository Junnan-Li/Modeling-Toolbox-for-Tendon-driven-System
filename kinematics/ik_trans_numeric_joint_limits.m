% numeric inverse kinematic methods: inverse Jacobian 
% 
% 
% 
% TODO: 
%   base rotation and position not considered



function [q,q_all,x_res,phi_x,iter] = ik_trans_numeric_joint_limits(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha)
%#codegen


q_i = q_init;
n_q = length(q_i);

q = zeros(n_q,1);
q_all = zeros(iter_max,n_q);
x_res = zeros(3,1);
phi_x = zeros(3,1);
x_i = zeros(3,1);
delta_x_i = zeros(3,1);
iter = 0;

% mdh_matrix = mdh_struct_to_matrix(mdh, 1);
mdh_q = mdh_matrix;
for i = 1:iter_max
    q_all(i,:) = q_i;
    mdh_q(1:n_q,3) = mdh_matrix(1:n_q,3) + q_i;
    T = T_mdh_multi(mdh_q);
    x_i = T(1:3,4);
    delta_x_i = x_des - x_i;
    if abs(delta_x_i) < tol
        break
    end
    J = Jacobian_analytic_mdh(mdh_matrix,q_i);
    J_trans = J(1:3,:);

    delta_q = alpha * pinv(J_trans) * delta_x_i;
    q_i_new = q_i + delta_q; 
    q_i = max(min(q_i_new, q_limits(:, 2)), q_limits(:, 1));
end

x_res = x_i;
phi_x = delta_x_i;
q = q_i;
iter = i;
end

