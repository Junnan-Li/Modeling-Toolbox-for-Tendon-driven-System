% general Geometric Jacobian of a given point. The link information is
% separated as base and fingers
% 
% 
% Input:
%           w_T_all_links:       [4,4,nq] 
%           n_links:             [1,2]      number of bases, number of fingers
%           q_index:             [nb+nf,2]  index of q for each base/finger
%           index_link           [1]        index of links
%           w_p_point:           [3,1]      position of the point in world frame
%   
% Output:
%           J: [6,nq] Jacobian matrix 
% 
% %  Not finished yet


function J = sim_w_Jacobian_geom_from_T_links(w_T_all_links, n_links, q_index, index_link, w_p_point)

n_q = size(w_T_all_links,3);
n_b = n_links(1); % number of bases
% n_f = n_links(2); % number of fingers
% njb = q_index(1:n_b,2)-q_index(1:n_b,1) + 1; % number of joints of each base
% njf = q_index((1+n_b):(n_b+n_f),2)-q_index((1+n_b):(n_b+n_f),1) + 1; % number of joints of each finger


J = zeros(6,n_q);
% w_p_point = T_p31(w_T_all(:,:,index_link),reshape(i_p_point,3,1));

if index_link <= 0
    return
end

i_link = max(find((index_link - q_index(:,1))>=0));% point in ith link
if i_link <= n_b % the point is on the base not finger
    for i = 1:index_link
        w_T_i = w_T_all_links(:,:,i);
        w_i_p_point = w_p_point(1:3)-w_T_i(1:3,4);
        J_tmp_p = cross(w_T_i(1:3,1:3)*[0;0;1],w_i_p_point);
        J_tmp_w = w_T_i(1:3,1:3)*[0;0;1];
        J(:,i) = [J_tmp_p;J_tmp_w];
    end
else
    for i = 1:q_index(n_b,2) % all joints on the base
        w_T_i = w_T_all_links(:,:,i);
        w_i_p_point = w_p_point(1:3)-w_T_i(1:3,4);
        J_tmp_p = cross(w_T_i(1:3,1:3)*[0;0;1],w_i_p_point);
        J_tmp_w = w_T_i(1:3,1:3)*[0;0;1];
        J(:,i) = [J_tmp_p;J_tmp_w];
    end
    for i = q_index(i_link,1):index_link
        w_T_i = w_T_all_links(:,:,i);
        w_i_p_point = w_p_point-w_T_i(1:3,4);
        J_tmp_p = cross(w_T_i(1:3,1:3)*[0;0;1],w_i_p_point);
        J_tmp_w = w_T_i(1:3,1:3)*[0;0;1];
        J(:,i) = [J_tmp_p;J_tmp_w];
    end
end
end