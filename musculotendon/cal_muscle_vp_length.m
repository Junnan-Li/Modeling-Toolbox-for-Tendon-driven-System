% calculate the muscle length with a given set of viapoints  
% 
% Input:
%           p_vp:   [3xn] position of n via points
% Output:
%           l:      [1] length of the muscle 
% 
% Reference:
% 
% Junnan Li, junnan.li@tum.de, 07.2024


function l = cal_muscle_vp_length(p_vp)

n_vp = size(p_vp,2);
l = 0;
for i = 1:n_vp-1
    l = l + norm(p_vp(:,i+1)-p_vp(:,i));
end
end