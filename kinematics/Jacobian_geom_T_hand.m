% general Geometric Jacobian of a given point. The link information is
% separated as base and fingers
% 
% 
% Input:
%           w_T_all: [4,4,nl]
%           index_base: [1,2]  [1, index_last_base]
%           index_finger: [nf,2]    [index_start_finger_link, index_last_finger_link]
%           index_link: [1] the index of link that the point is attached
%           w_p_point:  position of the point in world frame
%   
% Output:
%           J: [6,n_q] Jacobian matrix 
% 
% Junnan Li, junnan.li@tum.de, 08.2024


function J = Jacobian_geom_T_hand(w_T_all, index_base, index_finger, index_link, w_p_point)

nl = size(w_T_all,3);
assert(index_base(2) <= nl, '[Jacobian_geom_T_sharedbase]: input index_base is incorrect! ')
assert(index_link <= nl, '[Jacobian_geom_T_sharedbase]: input index_base is incorrect! ')
assert(length(w_p_point) == 3, '[Jacobian_geom_T_sharedbase]: input index_base is incorrect! ')
J = zeros(6,nl);
% w_p_point = T_p31(w_T_all(:,:,index_link),reshape(i_p_point,3,1));


for i = 1:index_base(2)
    w_T_i = w_T_all(:,:,i);
    w_i_p_point = w_p_point(1:3)-w_T_i(1:3,4);
    J_tmp_p = cross(w_T_i(1:3,1:3)*[0;0;1],w_i_p_point);
    J_tmp_w = w_T_i(1:3,1:3)*[0;0;1];
    J(:,i) = [J_tmp_p;J_tmp_w];
    if index_link == i
        return
    end
end

i_link = max(find((index_link-index_finger(:,1))>=0));% point in ith link

for i = index_finger(i_link,1):index_finger(i_link,2)
    w_T_i = w_T_all(:,:,i);
    w_i_p_point = w_p_point-w_T_i(1:3,4);
    J_tmp_p = cross(w_T_i(1:3,1:3)*[0;0;1],w_i_p_point);
    J_tmp_w = w_T_i(1:3,1:3)*[0;0;1];
    J(:,i) = [J_tmp_p;J_tmp_w];
    if index_link == i
        return
    end
end

end