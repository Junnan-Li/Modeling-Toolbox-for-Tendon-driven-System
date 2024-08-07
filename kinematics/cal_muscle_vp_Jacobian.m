% calculate the Jacobian of length to jont angle with given n points  
% 
% Input:
%           J_vp    [3,nj,n]
%           p_vp:   [3xn] position of n via points
% Output:
%           J:      [nj,1] length of the muscle 
% 
% Reference:
% 
% Junnan Li, junnan.li@tum.de, 07.2024


function J = cal_muscle_vp_Jacobian(w_J_vp, w_p_vp)

n_vp = size(w_p_vp,2);
nj = size(w_J_vp,2);
assert(size(w_p_vp,1)==3,'[cal_muscle_vp_Jacobian]: input dimension is incorrect!')
assert(size(w_J_vp,3)==n_vp ,'[cal_muscle_vp_Jacobian]: input dimension is incorrect!')


J = zeros(nj,1);
for i = 1:n_vp-1
    w_p_i = w_p_vp(:,i+1) - w_p_vp(:,i);
    w_p_i_norm = w_p_i/norm(w_p_i);
    J_i =  (w_J_vp(:,:,i+1) - w_J_vp(:,:,i))' * w_p_i_norm;
    J = J + J_i;
end

end