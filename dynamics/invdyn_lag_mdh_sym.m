% calculate Mass matrix using Lagrangian Euler method
% Input:
%     q:                [n_qx1] joint states
%     mdh:              [n_q+1x4] in sequence of [alpha,a,theta,d] To be
%                       noted that the theta should exclude from mdh parameters 
%     Mass:             [n_q+1] mass vector. the first mass is the base 
%     CoM:              [3xn_q+1] in local frame; last column is endeffector
%     I:                [6xn_q+1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
%     g:                [3x1] gravity term in world frame
% 


% Comment: [01/24] first version: base is fixed, no external force is
% needed 


function [Tau, M, C, G] = invdyn_lag_mdh_sym(q, qd, qdd, mdh, x_base, Mass, CoM, I, g) % , X_base, XD_base,XDD_base 
%#codegen



n_q = length(q);
% assert(n_q == length(qD)&&n_q == length(qDD),'dimension error!');
g = reshape(g,[3,1]);

q_sym = sym('q_tmp',[n_q,1], 'real');

M_sym = invdyn_lag_mdh_M_sym(q_sym, mdh, x_base, Mass, CoM, I); % symbolic representation
M = subs(M_sym,q_sym,q);
% christoffel symbol
C = sym(zeros(n_q,n_q));
for i = 1:n_q
    for j = 1:n_q
        for k = 1:n_q
            h_ijk_sym = diff(M_sym(i,j),q_sym(k)) - 1/2*diff(M_sym(j,k),q_sym(i));
            h_ijk_sym = simplify(h_ijk_sym);
            h_ijk = subs(h_ijk_sym,q_sym,q);
            C(i,j) = C(i,j) + h_ijk*qd(k);
        end
        C(i,j) = simplify(C(i,j));
    end
end

G = invdyn_lag_mdh_G_sym(q, mdh, x_base, Mass, CoM, g);
Tau = M*qdd + C*qd + G;


