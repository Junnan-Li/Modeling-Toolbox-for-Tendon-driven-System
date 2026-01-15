% Transformation matrix from mdh parameter
% 
% Input:
%           mdh: as the order: alpha,a,theta,d
% Output:
%           T: [4x4] Transformation matrix {n-1}^T_{n}
% 
% Source:     
%       Wiki: https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters


function T = T_mdh_single(alpha,a,theta,d)

% assert(length(alpha)== obj.nja, 'dimension of joint vector is incorrect!')

T = [cos(theta),           -sin(theta),            0,           a;
    sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
    sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
    0,                     0,                      0,           1];

end

