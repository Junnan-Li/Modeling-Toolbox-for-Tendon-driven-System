% Geometric Jacobian of the end-effector based on the jonit cofiguration
% 
% Input:
%           mdh: [(n_q+1)x4] in the order: alpha,a,theta,d
%           q:   [n_q]
% Output:
%           J: [6xn_q] Transformation matrix {i}^T_{j}
% 
% Junnan Li, junnan.li@tum.de, 02.2023


function J = Jacobian_geom_mdh(mdh,q)

n_q = length(q);
J = zeros(6,n_q);

mdh(1:n_q,3) = mdh(1:n_q,3) + q;

base_T_end = T_mdh_multi(mdh);

for i = 1:n_q
    
    base_T_i = T_mdh_multi(mdh(1:i,:));
    b_R_i = base_T_i(1:3,1:3);
    i_T_end = base_T_i\base_T_end;
    i_p_end = i_T_end(1:3,4);
    J_tmp_p = cross(b_R_i*[0;0;1],b_R_i*i_p_end);
    J_tmp_w = b_R_i*[0;0;1];
    J(:,i) = [J_tmp_p;J_tmp_w];
end

end