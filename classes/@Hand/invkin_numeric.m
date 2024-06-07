% Numeric inverse kinematic method for a hand 
% 
% input:
%           x_des_all: desired position in world frame of al fingertips
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

function [q_res, q_all, x_res,phi_x,iter] = invkin_numeric(obj,x_des_i,finger_index,iter_max,tol,alpha)

if nargin == 3
    iter_max = 100;
    tol = [1e-9;1e-6];
    alpha = 0.1;
elseif nargin == 4
    tol = [1e-9;1e-6];
    alpha = 0.1;
elseif nargin == 5
    alpha = 0.1;
end
assert(all(size(x_des_i) == [6,1]), 'Hand.invkin_numeric: wrong dimension of X_des_all')

q_all = zeros(iter_max,obj.nj);
q_res = zeros(size(obj.nj));
% figure(2)
% plot3(x_des_all(1),x_des_all(2),x_des_all(3),'*','MarkerSize',15)
hold on
for i = 1:iter_max
    x_all_i = obj.get_w_x_ee_all();
    x_i = x_all_i(:,finger_index);
    q_i = obj.q;
    q_all(i,:) = q_i;
    delta_x_i = x_des_i - x_i;
    if max(abs(delta_x_i(1:3))) < tol(1) && max(abs(delta_x_i(4:6))) < tol(2)
        break
    end
    J_w = obj.Jacobian_ana_w_one_finger(finger_index,q_i);
    delta_q = alpha*pinv(J_w) * delta_x_i;
    q_i_new = q_i + delta_q;
    obj.update_hand(q_i_new);
%     obj.plot_hand
%     drawnow
end
q_res = q_i;
x_res = x_i;
phi_x = delta_x_i;
iter = i;


end

