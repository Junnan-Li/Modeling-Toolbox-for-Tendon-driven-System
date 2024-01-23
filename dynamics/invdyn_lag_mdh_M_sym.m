% calculate Mass matrix using Lagrangian Euler method
% Input:
%     q:         [n_qx1] joint states
%     mdh:              [n_q+1x4] in sequence of [alpha,a,theta,d] To be
%                       noted that the theta should exclude from mdh parameters 
%     Mass:             [n_q+1] mass vector. the first mass is the base 
%     CoM:              [3xn_q+1] in local frame; last column is endeffector
%     I:                [6xn_q+1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
%     g:                [3x1] gravity term in world frame
% 


% Comment: [01/24] first version: base is fixed, no external force is
% needed 


function M = invdyn_lag_mdh_M_sym(q, mdh, x_base, Mass, CoM, I) % , X_base, XD_base,XDD_base 
%#codegen

n_q = length(q);
M = sym(zeros(n_q,n_q));
mdh_q = mdh;
mdh_q(1:end-1,3) = mdh_q(1:end-1,3) + q;

w_T_base = sym([euler2R_XYZ(x_base(4:6)),x_base(1:3);...
                0 0 0 1]);

base_T_all = sym(zeros(4,4*n_q),'d');
for i = 1:n_q
    base_T_all(:,4*i-3:4*i) = T_mdh_multi(mdh_q(1:i,:)); % transformation to each frame
end
base_T_all = simplify(base_T_all);
w_T_all = w_T_base*base_T_all;


for i = 1:n_q
    % for each frame/link, base is fixed,  start from the first link
    w_T_i = w_T_all(:,4*i-3:4*i);
    w_R_i = w_T_i(1:3,1:3);
    w_p_i = w_T_i(1:3,4);
    w_p_mi = w_p_i + w_R_i * CoM(:,i+1); % pos of com_i in base frame
    w_p_mi = simplify(w_p_mi);
    Jt_i = sym(zeros(3,n_q));
    Jr_i = sym(zeros(3,n_q));
    for j = 1:i % iteration from 1 to i
        w_T_j = w_T_all(:,4*j-3:4*j);
        w_R_j = w_T_j(1:3,1:3);
        w_p_j = w_T_j(1:3,4);
        Jt_ij = cross(w_R_j*[0;0;1],w_p_mi-w_p_j); % translational J
        Jt_ij = simplify(Jt_ij);
        Jr_ij = w_R_j*[0;0;1]; % rotational J
        Jr_ij = simplify(Jr_ij);
        Jt_i(:,j) = Jt_ij;
        Jr_i(:,j) = Jr_ij;
    end
    
    I_i = inertia_tensor2matrix(I(:,i+1)); % matrix of Inertia
    M_i = Mass(i+1) * Jt_i'*Jt_i + Jr_i'*w_R_i*I_i*w_R_i'*Jr_i; % calculate Mass matrix
    M_i = simplify(M_i);
    
    M = M + M_i;

end
M = simplify(M);


