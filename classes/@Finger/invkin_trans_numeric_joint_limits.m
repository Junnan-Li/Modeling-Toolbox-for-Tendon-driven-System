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

function [q_res, q_all, x_res,phi_x,iter] = invkin_trans_numeric_joint_limits(obj,x_des,iter_max,tol,alpha, mex)

if nargin == 2
    iter_max = 100;
    tol = 1e-9;
    alpha = 0.8;
    mex = 0;
elseif nargin == 3
    tol = 1e-9;
    alpha = 0.8;
    mex = 0;
elseif nargin == 4
    alpha = 0.8;
    mex = 0;
elseif nargin == 5
    mex = 0;
end
% assert(length(q)== obj.nja, 'dimension of joint vector is incorrect!')

q_all = zeros(iter_max,obj.nja);
q_res = zeros(size(obj.nja));


q_limits = obj.limits_q(:,1:2);
% for j = 1:obj.nja
%     a(j) = (q_limits(j,2)+q_limits(j,1))/2;  
% end

w_p_base = obj.w_p_base;
w_R_base = obj.w_R_base;
% w_T_base = [obj.w_R_base,obj.w_p_base;0,0,0,1];


if mex
    q_init = obj.q_a;
    mdh_matrix = mdh_struct_to_matrix(obj.mdh_ori, 1);
    
    x_des_mod = w_R_base' * (x_des - w_p_base);
    
    [q_mex,q_all,x_mex,phi_mex,iter_mex] = ik_trans_numeric_joint_limits_mex(mdh_matrix,x_des_mod,q_init,q_limits,iter_max,tol,alpha);
    
    obj.update_finger(q_mex);
    q_res = q_mex;
    x_res = w_R_base * x_mex + w_p_base;
    phi_x = w_R_base * phi_mex;
    iter = iter_mex; 
else
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
        
        %%% test methods
        %     for j = 1:obj.nja
        %        dH(j) = (q_i(j)-a(j))/((a(j)-q_limits(j,2))^2);
        %     end
        %     NJ = -0.1*(eye(obj.nja)-pinv(J_trans)*J_trans)*dH;
        %     delta_q = alpha * pinv(J_trans) * delta_x_i + NJ;
        
        delta_q = alpha * pinv(J_trans) * delta_x_i;
        q_i_new = q_i + delta_q;
        q_i_new_sat = max(min(q_i_new, q_limits(:, 2)), q_limits(:, 1));
        obj.update_finger(q_i_new_sat);
    end
    q_res = q_i;
    x_res = x_i;
    phi_x = delta_x_i;
    iter = i;
end

end

