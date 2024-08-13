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
%           wrap_status {nmus}
%           w_PS_p      {nmus}
% 




function [length,J, wrap_status,w_PS_p] = cal_hand_Muscle_l_J_Garner_old(obj, varargin)
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

% initialize
length = zeros(nmus,1);
J = zeros(nj,nmus);
wrap_status = cell(nmus,1);
w_PS_p = cell(nmus,1);


for i_mus = 1:nmus
    muscle_i = obj.list_muscles(i_mus);
    if ~isempty(muscle_i.constr_vp)
%         l_vp = nan(length(muscle_i.constr_vp),1);
        for i = 1:size(muscle_i.constr_vp,1)
            %  path only constrained by via points
            constr_vp_i = muscle_i.constr_vp{i};
            n_vp_i = size(constr_vp_i,1); % number of via points
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
%         l_obs = nan(length(muscle_i.constr_obs),1);
%         w_PS_p_i = cell(length(muscle_i.constr_obs),1);
%         wrap_status_i = nan(length(muscle_i.constr_obs),1);
        for i = 1:size(muscle_i.constr_obs,1)
            constr_obs_i = muscle_i.constr_obs{i};
            if size(constr_obs_i,1) == 3
                w_P_p = constr_obs_i{1}.get_w_p_VP_inhand;
                w_T_obs = constr_obs_i{2}.get_w_T_Obs_inhand;
                w_S_p = constr_obs_i{3}.get_w_p_VP_inhand;
                radius = constr_obs_i{2}.radius;
                wrap_direction = sign(constr_obs_i{2}.axis(3));
                [l_QT_i,wrap_status_i,w_Q_p,w_T_p] = cal_obstacle_vp_cyl_Garner(w_T_obs, w_P_p, w_S_p, radius, wrap_direction);

                w_J_P = obj.Jacobian_geom_w_vp(constr_obs_i{1});
                w_J_S = obj.Jacobian_geom_w_vp(constr_obs_i{3});
                if wrap_status_i
                    J_vp = zeros(3,nj,4);
                    w_T_link = constr_obs_i{2}.Link.w_T_Link_inhand;
                    w_J_Q = obj.Jacobian_geom_w_point(constr_obs_i{2}.index_link_inhand, w_Q_p);
                    w_J_T = obj.Jacobian_geom_w_point(constr_obs_i{2}.index_link_inhand, w_T_p);
                    J_vp(:,:,1) = w_J_P;
                    J_vp(:,:,2) = w_J_Q;
                    J_vp(:,:,3) = w_J_T;
                    J_vp(:,:,4) = w_J_S;
                    J_lq_constr_obs_i_l1 = cal_muscle_vp_Jacobian(J_vp(:,:,1:2), [w_P_p,w_Q_p]);
                    J_lq_constr_obs_i_l2 = cal_muscle_vp_Jacobian(J_vp(:,:,3:4), [w_T_p,w_S_p]);
                    J_lq_constr_obs_i = J_lq_constr_obs_i_l1 + J_lq_constr_obs_i_l2;
                    w_PS_p{i_mus,i} = [w_Q_p,w_T_p];
                else
                    J_vp = zeros(3,nj,2);
                    J_vp(:,:,1) = w_J_P;
                    J_vp(:,:,2) = w_J_S;
                    J_lq_constr_obs_i = cal_muscle_vp_Jacobian(J_vp, [w_P_p,w_S_p]);
                    w_PS_p{i_mus,i} = nan;
                end
            else
                disp('[Muscle.cal_Muscle_length_ObstacleSet_Cyl_Garner]: constr_obs has more than 3 elements! ')
            end
            wrap_status{i_mus,i} = wrap_status_i;
            length(i_mus) = length(i_mus) + l_QT_i;
            J(:,i_mus) = J(:,i_mus) + J_lq_constr_obs_i;
        end
    end
end

if nargin > 1
    obj.update_hand(q_init);
end
end