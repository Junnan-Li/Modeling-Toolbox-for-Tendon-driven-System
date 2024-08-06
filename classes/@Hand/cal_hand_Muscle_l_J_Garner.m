% calculate all muscle length, Jacobian, wrapping condition and position of 
% wrapping via points of a hand object
% 
% comments
%       [06/08] current version only support cylinder obstacles, 
%               calculation of Jacobian of vp is not optimized (repetitive)
% 
% obj.nmus:     number of muscles
% obj.nj:       number of joints (frames)
% 
% 
% input:
%           obj 
%           varargin
%               q
% 
% output:
%           length:     [nmus] length of all muscles 
%           J           [njxnmus] Jacobian matrix of lentgh to joint angle
%                                   (moment arm matrix)
%           
% 
% 




function [length,J, wrap_status,w_PS_p] = cal_hand_Muscle_l_J_Garner(obj, varargin)
% calculate the muscle length with Cylinder obstacle set
% using cal_obstacle_vp_cyl_Garner.m
if nargin <= 1
    q = obj.q;
else
    q = varargin{1};
    q = reshape(q,[obj.nj,1]);
    obj.update_hand(q);
end


nmus = obj.nmus;
nj = obj.nj;

w_T_links_all = obj.get_w_T_links_inhand;
assert(all(size(w_T_links_all)==[4,4,obj.nj]),'[cal_hand_Muscle_l_J_Garner] dimension of w_T_links_all is incorrect!')

length = nan(nmus,1);
J = nan(nj,nmus);
wrap_status = nan(nmus,1);

for i_mus = 1:nmus
    muscle_i = obj.list_muscles(i_mus);
    if ~isempty(muscle_i.constr_vp)
%         l_vp = nan(length(muscle_i.constr_vp),1);

        for i = 1:length(muscle_i.constr_vp)
            %  path only constrained by via points
            constr_vp_i = muscle_i.constr_vp{i};
            n_vp_i = length(constr_vp_i); % number of via points
            w_p_vp = nan(3,n_vp_i);
            J_vp = nan(3,nj,n_vp_i);
            for vp_index = 1:n_vp_i
                vp_i = constr_vp_i{vp_index};
                w_p_vp(:,vp_index) = vp_i.get_w_p_VP_inhand;
                J_vp(:,:,vp_index) = obj.Jacobian_geom_w_vp(vp_i);
            end
            J_constr_vp_i = cal_muscle_vp_Jacobian(J_vp, w_p_vp);
            l_constr_vp_i = cal_muscle_vp_length(w_p_vp);
            length(i_mus) = length(i_mus) + l_constr_vp_i;
            J(:,i_mus) = J(:,i_mus) + J_constr_vp_i;
        end
    end
    if ~isempty(muscle_i.constr_obs)
        l_obs = nan(length(muscle_i.constr_obs),1);
        w_PS_p = cell(length(muscle_i.constr_obs),1);
        wrap_status_i = nan(length(muscle_i.constr_obs),1);
        for i = 1:length(muscle_i.constr_obs)
            constr_obs_i = muscle_i.constr_obs{i};
            if length(constr_obs_i)==3
                if in_hand == 0
                    w_P_p = constr_obs_i{1}.get_w_p_VP;
                    w_T_obs = constr_obs_i{2}.get_w_T_Obs;
                    w_S_p = constr_obs_i{3}.get_w_p_VP;
                else
                    w_P_p = constr_obs_i{1}.get_w_p_VP_inhand;
                    w_T_obs = constr_obs_i{2}.get_w_T_Obs_inhand;
                    w_S_p = constr_obs_i{3}.get_w_p_VP_inhand;
                end
                radius = constr_obs_i{2}.radius;
                wrap_direction = sign(constr_obs_i{2}.axis(3));
                [l_QT_i,wrap_status_i(i),w_Q_p,w_T_p] = cal_obstacle_vp_cyl_Garner(w_T_obs, w_P_p, w_S_p, radius, wrap_direction);
                
                if wrap_status_i(i)
                    w_PS_p{i,1} = [w_Q_p,w_T_p];
                end
            else
                disp('[Muscle.cal_Muscle_length_ObstacleSet_Cyl_Garner]: constr_obs has more than 3 elements! ')
            end
            wrap_status(i_mus) = wrap_status_i;
            l_obs(i) = l_QT_i;
            length(i_mus) = length(i_mus) + l_QT_i;
        end
    end
end

if nargin > 1
    obj.update_hand(q_init);
end
end