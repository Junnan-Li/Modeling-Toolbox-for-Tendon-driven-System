% generate the symbolic expression of transformation matrices of all links
% with q as input.
% 
% input:
%           w_T_b:      [4x(nb+nf),4] transformation matrix of each finger base
%           mdh:        [nx4]   mdh parameters of all links
%           mdh_index:  [nlx2]  index of start and end row of each finger
%                               in mdh matrix
%           nb:         [1]     number of base finger of a hand
%           q:          [nj]    joint angle
% 
% output:
%           w_T_all_links: [4,4,nl] transformation matrix for all links 
% 
%   


function w_T_all_links = sim_w_T_all_links_from_q(mdh,mdh_index,q_index,w_T_b,n_links,q)
%#codegen

nl = length(q);
nb = n_links(1);
nf = n_links(2);
% assert(size())

% get all transformation matrix of all links
w_T_b_prior = eye(4);
w_T_all_links = zeros(4,4,nl);% 

link_index = 1;
if nb ~= 0
    for i = 1:nb
        mdh_i = mdh(mdh_index(i,1):mdh_index(i,2),:);
        mdh_i(:,3) = mdh_i(:,3) + [q(q_index(i,1):q_index(i,2));0];
        w_T_bi = w_T_b(4*i-3:4*i,:);
        w_T_bi_inhand = w_T_b_prior*w_T_bi;
        nl_i = size(mdh_i,1)-1;
        % update all link properties
        for j = 1:nl_i
            b_T_i = T_mdh_multi(mdh_i(1:j,:));
            w_T_all_links(:,:,link_index) = w_T_bi_inhand*b_T_i;
            link_index = link_index+1;
        end
        % w_T_b_prior is the base transformation for the next finger
        w_T_b_prior = w_T_bi_inhand * T_mdh_multi(mdh_i);
    end
end
if nf ~= 0
    for i = nb+1 : nb+nf
        mdh_i = mdh(mdh_index(i,1):mdh_index(i,2),:);
        mdh_i(:,3) = mdh_i(:,3) + [q(q_index(i,1):q_index(i,2));0];
        w_T_bi = w_T_b(4*i-3:4*i,:);
        w_T_bi_inhand = w_T_b_prior * w_T_bi;
        nl_i = size(mdh_i,1)-1;
        % update all link properties
        for j = 1:nl_i
            b_T_i = T_mdh_multi(mdh_i(1:j,:));
            w_T_all_links(:,:,link_index) = w_T_bi_inhand*b_T_i;
            link_index = link_index+1;
        end
    end
end

end