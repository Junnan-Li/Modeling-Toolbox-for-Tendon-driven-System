% generate the symbolic expression of transformation matrices of all frames
% with q as input.
% 
% input:
%           mdh:        [nx4]   mdh parameters of all links
%           mdh_index:  [nlx2]  index of start and end row of each finger
%                               in mdh matrix
%           q_index:    
%           w_T_b:      [4x(nb+nf),4] transformation matrix of each finger base
%           n_links:    [1]     number of base finger of a hand
%           q:          [nj]    joint angle
% 
% Output:   
%           w_vp: 
%           w_J_vp
% 
% 

function [w_vp, w_J_vp] = sim_w_vp_from_T_links(w_T_all_links, n_links, q_index,sim_vp_index, sim_vp_pos)
%#codegen

n_q = size(w_T_all_links,3);
n_b = n_links(1);
nvia = length(vp_index);
w_vp = nan(nvia,3);
w_J_vp = zeros(3,n_q);

for i = 1:nvia
    vp_index_i = sim_vp_index(i);
    i_vp_pos_i = sim_vp_pos(i,:);
    w_T_link_i = w_T_all_links(4*i-3:4*i,4); 
    w_vp(i,:) = T_p31(w_T_link_i, i_vp_pos_i)';




end


end