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
% Junnan Li, junnan.li@tum.de, 04.2024

function [q_res, q_all, x_res,phi_x,iter] = invkin_numeric(obj,x_des,iter_max,tol,alpha)

if nargin == 2
    iter_max = 100;
    tol = [1e-9;1e-6];
    alpha = 0.15;
elseif nargin == 3
    tol = [1e-9;1e-6];
    alpha = 0.15;
elseif nargin == 4
    alpha = 0.15;
end

q_all = zeros(iter_max,obj.nja);
q_res = zeros(size(obj.nja));
figure(2)
plot3(x_des(1),x_des(2),x_des(3),'*','MarkerSize',15)
hold on
for i = 1:iter_max
    x_i = obj.get_x_ee_w();
    q_i = obj.q_a;
    q_all(i,:) = q_i;
    delta_x_i = x_des - x_i;
    if max(abs(delta_x_i(1:3))) < tol(1) && max(abs(delta_x_i(4:6))) < tol(2)
        break
    end
    J = obj.Jacobian_analytic_b_end;
    J_w = blkdiag(obj.w_R_base,obj.w_R_base) * J;
    delta_q = alpha*pinv(J_w) * delta_x_i;
    q_i_new = q_i + delta_q;
    obj.update_finger(q_i_new);
    obj.plot_finger
    drawnow
end
q_res = q_i;
x_res = x_i;
phi_x = delta_x_i;
iter = i;


end

