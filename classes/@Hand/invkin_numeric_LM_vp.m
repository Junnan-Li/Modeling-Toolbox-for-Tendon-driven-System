% Numeric inverse kinematic method for a hand (Levenburg Marquadt) for
% given viapoints
%
% input:
%           x_vp_des: [3,N] desired position in world frame of one fingertips [3,1]
%           vp_list: [N] index of finger
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

function [q_res, info] = invkin_numeric_LM_vp(obj,x_vp_des,vp_list,varargin)

if nargin == 3
    ikpar = IK_par();
elseif nargin == 4
    ikpar = varargin{1};
else
    error('[invkin_numeric_LM_vp]: input dimension is incorrect! ')
end
num_vp = length(vp_list);
x_vp_des_vec = reshape(x_vp_des,3*num_vp,1);
assert(length(x_vp_des(:) == 3*num_vp), '[invkin_numeric_LM_vp]: wrong dimension of X_des_all')
x_vp_des = reshape(x_vp_des,3,num_vp);
par = ikpar.ikpar_LM;
W_e = par.W_e*eye(3*num_vp);
W_d = par.W_d*eye(obj.nj);
iter_max = par.iter_max;
tol = par.tol;
retry_num = par.retry_num;


q_all = zeros(iter_max,obj.nj);
q_res = zeros(size(obj.nj));

info = struct();
info.status = 0;
info.x_res = zeros(size(x_vp_des));
info.phi_x = zeros(size(x_vp_des));
info.iter = 0;
info.retry_iter = 0;

if par.visual % visualization
    figure()
end

for retry_i = 0:retry_num
    try
        for i = 1:iter_max
            x_vp_all_i = obj.get_p_all_viapoints_inhand();
            x_i_vec = reshape(x_vp_all_i(:,vp_list),3*num_vp,1);
            q_i = obj.q;
            q_all(i,:) = q_i;
           
            if par.visual % visualization
                obj.plot_hand;
            end

            % error
            delta_x_i = x_vp_des_vec - x_i_vec; % [3*num_vp,1]

            if max(abs(delta_x_i)) < tol(1)
                info.status = 1;
                break
            end
%             J = obj.Jacobian_geom_w_vp(vp1)
            J = obj.Jacobian_geom_w_vp(obj.list_viapoints(vp_list));
            
            g_i = J'* W_e *delta_x_i;
            delta_q = inv(J'*W_e*J + W_d) * g_i;
            if max(abs(delta_q)) < par.tol_q
                info.status = 10;
                break
            end
            
            q_i_new = q_i + delta_q;
            % q_i_new_wrapped = mod(q_i_new + pi, 2*pi) - pi;

            % check q limits

            obj.update_hand(q_i_new);
        end
    catch ME
        disp('[Hand.invkin_numeric_LM_vp]: catch error!')
        %         obj.update_hand();
    end
    if info.status == 1 % IK solved
        break
    end
end
q_res = q_i;
info.x_res = x_vp_all_i(:,vp_list);
info.phi_x = reshape(delta_x_i,3,num_vp);
info.iter = i;
info.retry_iter = retry_i;
end

