% calculate Mass matrix using Lagrangian Euler method For finger object
% Input:

%     Mass:             [n_q+1] mass vector. the first mass is the base 
%     CoM:              [3xn_q+1] in local frame; last column is endeffector
%     I:                [6xn_q+1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
%     g:                [3x1] gravity term in world frame
% 


% Comment: [01/24] first version: base is fixed, no external force is
% needed 
% 
% tested in test_finger.m
% 


function M = invdyn_lag_T_M_Hand(w_T_all_frames, q_index, w_T_b, n_links,l_index_in_frame, Mass, CoM, I) % , X_base, XD_base,XDD_base 
%#codegen

n_b = n_links(1); % number of bases
n_f = n_links(2); % number of fingers
njb = q_index(1:n_b,2)-q_index(1:n_b,1) + 1; % number of joints of each base
njf = q_index((1+n_b):(n_b+n_f),2)-q_index((1+n_b):(n_b+n_f),1) + 1; % number of joints of each finger

n_q = q_index(end,2);
n_frame = n_q + 2*(n_b+n_f); % each Finger has nqi + 2 frames (except world frame) 

M = zeros(n_q,n_q);
% base joints
index_link = 1;


for base_index = 1:n_b
    for i = 1:njb(base_index) + 1 % include base 
        % for each frame/link, base is fixed,  start from the first link
        w_T_i = w_T_link_frames(:,4*i-3:4*i);
        w_R_i = w_T_i(1:3,1:3);
        w_p_i = w_T_i(1:3,4);
        w_p_mi = w_p_i + w_R_i * CoM(:,i+1); % pos of com_i in base frame
        
        J = sim_w_Jacobian_geom_from_T_links(w_T_link_frames, n_links, q_index, index_link, w_p_mi);
        
        I_i = inertia_tensor2matrix(I(:,i+1)); % matrix of Inertia
        M_i = Mass(i+1) * J'*J + J'*w_R_i*I_i*w_R_i'*J; % calculate Mass matrix
        M = M + M_i;

        index_link = index_link + 1;
    end
end




for index_f = 1:n_f
    M_i = zeros(q_index(n_b,2)+njf(index_f),q_index(n_b,2)+njf(index_f));

    for i = 1:q_index(n_b,2) % to the last joint of base
        % for each frame/link, base is fixed,  start from the first link
        w_T_i = w_T_link_frames(:,4*i-3:4*i);
        w_R_i = w_T_i(1:3,1:3);
        w_p_i = w_T_i(1:3,4);
        w_p_mi = w_p_i + w_R_i * CoM(:,i+1); % pos of com_i in base frame
        Jt_i = zeros(3,n_q);
        Jr_i = zeros(3,n_q);
        for j = 1:i % iteration from 1 to i
            w_T_j = w_T_link_frames(:,4*j-3:4*j);
            w_R_j = w_T_j(1:3,1:3);
            w_p_j = w_T_j(1:3,4);
            Jt_ij = cross(w_R_j*[0;0;1],w_p_mi-w_p_j); % translational J
            Jr_ij = w_R_j*[0;0;1]; % rotational J
            Jt_i(:,j) = Jt_ij;
            Jr_i(:,j) = Jr_ij;
        end
        I_i = inertia_tensor2matrix(I(:,i+1)); % matrix of Inertia
        M_i = Mass(i+1) * Jt_i'*Jt_i + Jr_i'*w_R_i*I_i*w_R_i'*Jr_i; % calculate Mass matrix
        M = M + M_i;
    end

end

