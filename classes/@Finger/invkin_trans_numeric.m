% Numeric inverse kinematic method 
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

function [q_res, q_all, x_res,phi_x,iter] = invkin_trans_numeric(obj,x_des,iter_max,tol,alpha)

if nargin == 2
    iter_max = 100;
    tol = 1e-9;
    alpha = 0.8;
elseif nargin == 3
    tol = 1e-9;
    alpha = 0.8;
elseif nargin == 4
    alpha = 0.8;
end
% assert(length(q)== obj.nja, 'dimension of joint vector is incorrect!')

q_all = zeros(iter_max,obj.nja);
q_res = zeros(size(obj.nja));
for i = 1:iter_max
    p_link_all_w_r = obj.get_p_all_links;
    q_i = obj.q_a;
    q_all(i,:) = q_i;
    x_i = p_link_all_w_r(:,end);
    delta_x_i = x_des - x_i;
    if abs(delta_x_i) < tol
        break
    end
    J = obj.Jacobian_analytic_b_end;
    J_trans = obj.w_R_base*J(1:3,:);
    
    delta_q = alpha*pinv(J_trans) * delta_x_i;
    q_i_new = q_i + delta_q;
    obj.update_finger(q_i_new);
end
q_res = q_i;
x_res = x_i;
phi_x = delta_x_i;
iter = i;


end

