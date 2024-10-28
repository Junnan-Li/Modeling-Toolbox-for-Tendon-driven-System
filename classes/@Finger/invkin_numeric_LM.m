% Numeric inverse kinematic method for a Finger (Levenberg-Marquardt)
% 
% input:
%           x_des: desired position in world frame
%           IK_par(optional)
% 
% output:
%           q: result of ik solution
%           info:
%               status: if problem solved
%               x_res: result of the position x
%               phi_x: PHI
%               iter: interation number 
%               retry_iter: num of retries
% 
% Junnan Li, junnan.li@tum.de, 04.2024

function [q_res, info] = invkin_numeric_LM(obj,x_des,varargin)

if nargin == 2
    ikpar = IK_par();
elseif nargin == 3
    ikpar = varargin{1};
else
    error('[Finger.invkin_ncumeric_LM]: input dimension is incorrect!')
end
assert(all(size(x_des) == [6,1]), '[Finger.invkin_numeric_LM]: wrong dimension of X_des')

par = ikpar.ikpar_LM;
W_e = par.W_e;
W_d = par.W_d*eye(obj.nj);
iter_max = par.iter_max;
tol = par.tol;
retry_num = par.retry_num;

q_all = zeros(iter_max,obj.nja);

info = struct();
info.status = 0;
info.x_res = zeros(size(x_des));
info.phi_x = zeros(size(x_des));
info.iter = 0;
info.retry_iter = 0;

if par.visual % visualization
    figure()
end

for retry_i = 0:retry_num
    try
        for i = 1:iter_max
            x_i = obj.get_x_ee_w();
            q_i = obj.q_a;
            q_all(i,:) = q_i;
            delta_x_i = x_des - x_i;
            if par.visual % visualization
                obj.plot_finger;
                drawnow
            end

            % error
            phi_p_i = x_des(1:3) - x_i(1:3);
            phi_R_i = euler2R_XYZ(x_des(4:6)) * euler2R_XYZ(x_i(4:6))';
            phi_R_i_eul = R2euler_XYZ(phi_R_i);
            delta_x_i = [phi_p_i;phi_R_i_eul];

            if max(abs(delta_x_i(1:3))) < tol(1) && max(abs(delta_x_i(4:6))) < tol(2)
                info.status = 1;
                break
            end
%             J = obj.Jacobian_analytic_b_end;
            J = obj.Jacobian_geom_b_end;
            J_w = blkdiag(obj.w_R_base,obj.w_R_base) * J;
            if rank(J_w) < min(size(J_w)) 
                disp('[Finger.invkin_numeric_LM]: Jacobian rank deficit')
            end
            g_i = J_w'* W_e *delta_x_i;
            delta_q = inv(J_w'*W_e*J_w + W_d) * g_i;
            q_i_new = q_i + delta_q;
            obj.update_finger(q_i_new);
        end
    catch ME
        disp('[Finger.invkin_numeric_LM]: catch error!')
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

