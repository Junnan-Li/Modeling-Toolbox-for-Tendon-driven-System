% Geometric Jacobian of the end-effector based on the jonit cofiguration
% 
% Input:
%           mdh: [(n_q+1)x4] in the order: alpha,a,theta,d
%           q:   [n_q]
% Output:
%           J: [6xn_q] Transformation matrix {i}^T_{j}
% 
% Junnan Li, junnan.li@tum.de, 02.2023


function J = Jacobian_analytic_mdh(mdh,q)

n_q = length(q);
J = zeros(6,n_q);

mdh_q = mdh;
mdh_q(1:n_q,3) = mdh_q(1:n_q,3) + q;

base_T_end = T_mdh_multi(mdh_q);
base_R_end = base_T_end(1:3,1:3);

J_geom = Jacobian_geom_mdh(mdh,q);

J = blkdiag(eye(3),base_R_end)*J_geom;


