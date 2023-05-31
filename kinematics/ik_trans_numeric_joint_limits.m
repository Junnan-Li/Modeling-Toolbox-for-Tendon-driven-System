% numeric inverse kinematic methods: inverse Jacobian 
% 
% 
% 
% TODO: 
%   base rotation and position not considered



function [q,x_res,phi_x,iter] = ik_trans_numeric_joint_limits(mdh,x_des,q_init,q_limits,iter_max,tol,alpha)



q_i = q_init;
n_q = length(q_i);
a = zeros(n_q,1);
dH = a;

for j = 1:n_q
    a(j) = (q_limits(j,2)+q_limits(j,1))/2;  %%第i个关节中值
end

mdh_matrix = mdh_struct_to_matrix(mdh, 1);
mdh_q = mdh_matrix;
for i = 1:iter_max
    mdh_q(1:n_q,3) = mdh_matrix(1:n_q,3) + q_i;
    T = T_mdh_multi(mdh_q);
    x_i = T(1:3,4);
    delta_x_i = x_des - x_i;
    if abs(delta_x_i) < tol
        break
    end
    J = Jacobian_analytic_mdh(mdh_matrix,q_i);
    J_trans = J(1:3,:);
    
    for j = 1:n_q
       dH(j) = 2*(q_i(j)-a(j))/((a(j)-q_limits(j,2))^2);
    end
    
    % joint limits 
    NJ = (eye(n_q)-pinv(J_trans)*J_trans)*dH;
    
    delta_q = alpha * pinv(J_trans) * delta_x_i + NJ;
    q_i = q_i + delta_q; 
end

x_res = x_i;
phi_x = delta_x_i;
q = q_i;
iter = i;
end

