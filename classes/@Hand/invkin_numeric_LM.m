% Numeric inverse kinematic method for a hand (Newton Raphson)
%
% input:
%           x_des_i: desired position in world frame of one fingertips [6,1]
%           finger_index: index of finger
%           par:
%               iter_max:   maximal iteration of each try
%               tol:        tolorence of each x
%               alpha:
%               retry_num:  retry iteration
%               visual
%
% output:
%           q: calculated joint angle
%           info
%               status:     success or not
%               x_res:      result of the position x
%               phi_x:      error of x
%               iter:       interation index
%
% Junnan Li, junnan.li@tum.de, 06.2024

function [q_res, info] = invkin_numeric_LM(obj,x_des_i,finger_index,varargin)

if nargin == 3
    ikpar = IK_par();
elseif nargin == 4
    ikpar = varargin{1};
else
    error('[invkin_numeric_LM]: input dimension is incorrect! ')
end
assert(all(size(x_des_i) == [6,1]), '[invkin_numeric_LM]: wrong dimension of X_des_all')

par = ikpar.ikpar_LM;
W_e = par.W_e;
W_d = par.W_d*eye(obj.nj);
iter_max = par.iter_max;
tol = par.tol;
retry_num = par.retry_num;

q_all = zeros(iter_max,obj.nj);
q_res = zeros(size(obj.nj));

info = struct();
info.status = 0;
info.x_res = zeros(size(x_des_i));
info.phi_x = zeros(size(x_des_i));
info.iter = 0;
info.retry_iter = 0;

if par.visual % visualization
    figure()
end

for retry_i = 0:retry_num
    try
        for i = 1:iter_max
            x_all_i = obj.get_w_x_ee_all();
            x_i = x_all_i(:,finger_index);
            q_i = obj.q;
            q_all(i,:) = q_i;

            if par.visual % visualization
                obj.plot_hand;
            end

            % error
            phi_p_i = x_des_i(1:3) - x_i(1:3);
            phi_R_i = euler2R_XYZ(x_des_i(4:6)) * euler2R_XYZ(x_i(4:6))';
            phi_R_i_eul = R2euler_XYZ(phi_R_i);
            delta_x_i = [phi_p_i;phi_R_i_eul];

            if max(abs(delta_x_i(1:3))) < tol(1) && max(abs(delta_x_i(4:6))) < tol(2)
                info.status = 1;
                break
            end
            J = obj.Jacobian_geom_w_one_finger(finger_index,q_i);

            if rank(J) < min(size(J))
                disp('[invkin_numeric_LM]: Jacobian rank deficit')
            end
            g_i = J'* W_e *delta_x_i;
            delta_q = inv(J'*W_e*J + W_d) * g_i;
            q_i_new = q_i + delta_q;
            obj.update_hand(q_i_new);
        end
    catch ME
        disp('[Hand.invkin_numeric_LM]: catch error!')
        %         obj.update_hand();
    end
    if info.status % IK solved
        break
    end
end
q_res = q_i;
info.x_res = x_i;
info.phi_x = delta_x_i;
info.iter = i;
info.retry_iter = retry_i;
end

