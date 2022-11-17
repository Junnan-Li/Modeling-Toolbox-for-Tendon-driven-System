% Transformation matrix from mdh parameters
% 
% Input:
%           mdh: [?x4]as the order: alpha,a,theta,d
% Output:
%           T: [4x4] Transformation matrix {i}^T_{j}
% 
% Source:     
%       Wiki: https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters


function T = T_mdh_multi(mdh)

assert(size(mdh,2)== 4, 'func T_mdh_multi error: dimension of mdh is incorrect!')

num_frame = size(mdh,1);
T = eye(4);

for i = 1:num_frame
    
    alpha = mdh(i,1);
    a = mdh(i,2);
    theta = mdh(i,3);
    d = mdh(i,4);
    
    T_tmp = [cos(theta),           -sin(theta),            0,           a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
        sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
        0,                     0,                      0,           1];
    T = T*T_tmp;
end


end

