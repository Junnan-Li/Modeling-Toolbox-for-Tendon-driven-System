% calculate Gravity torque (for compensation) using Lagrangian Euler method For Hand object with
% Transformation matrix as input 
% 
% Input:
%     w_T_all_frames    [4,4,nframe] transformation matrix of each frame
%     q_index         
%     n_links
%     l_index_in_frame
%     q_index_in_frame
% 
%     Mass:             [nlinks] mass vector. the first mass is the base 
%     CoM:              [3,n_q+1] in local frame; last column is endeffector
%     g:                [3,1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
% 
% 
% tested in test_simulation_functions.m
% link to invdyn_lag_T_G_Hand.m.template


function G = invdyn_lag_T_G_Hand(w_T_all_frames, q_index, n_links,l_index_in_frame,q_index_in_frame, Mass, CoM, g)  
%#codegen

n_b = n_links(1); % number of bases
n_f = n_links(2); % number of fingers
njb = q_index(1:n_b,2)-q_index(1:n_b,1) + 1; % number of joints of each base
njf = q_index((1+n_b):(n_b+n_f),2)-q_index((1+n_b):(n_b+n_f),1) + 1; % number of joints of each finger

n_q = q_index(end,2);

G = zeros(n_q,1);
w_T_q_frames = w_T_all_frames(:,:,q_index_in_frame);
w_T_links_frames = w_T_all_frames(:,:,l_index_in_frame);
% 
index_q = 0; % counting index of q
index_l = 0; % counting index of Link/base 

for base_index = 1:n_b
    for i = 1:njb(base_index) + 1 % include base 
        % for each frame/link, base is fixed,  start from the first link
        if i ~= 1
            index_q = index_q + 1; % the first link of each Finger is base
        end
        index_l = index_l + 1;
        w_T_i = w_T_links_frames(:,:,index_l);
        w_R_i = w_T_i(1:3,1:3);
        w_p_i = w_T_i(1:3,4);
        w_p_mi = w_p_i + w_R_i * CoM(:,index_l); % pos of com_i in base frame
        J = sim_w_Jacobian_geom_from_T_links(w_T_q_frames, n_links, q_index, index_q, w_p_mi);
        J_t = J(1:3,:);
        G_i = Mass(index_l) * J_t' * g; % calculate Mass matrix
        G = G - G_i;    
    end
end

for finger_index = 1:n_f
    for i = 1:njf(finger_index) + 1 % include base 
        index_l = index_l + 1;
        w_T_i = w_T_links_frames(:,:,index_l);
        w_R_i = w_T_i(1:3,1:3);
        w_p_i = w_T_i(1:3,4);
        w_p_mi = w_p_i + w_R_i * CoM(:,index_l); % pos of com_i in base frame
        if i == 1 % the first link of each Finger is base
            J = sim_w_Jacobian_geom_from_T_links(w_T_q_frames, n_links, q_index, q_index(n_b,2), w_p_mi);
        else
            index_q = index_q + 1;
            J = sim_w_Jacobian_geom_from_T_links(w_T_q_frames, n_links, q_index, index_q, w_p_mi);
        end
        J_t = J(1:3,:);
        G_i = Mass(index_l) * J_t' * g; % calculate Mass matrix
        G = G - G_i;    
    end
end

