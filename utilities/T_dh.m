% Transformation matrix from dh parameter
% 
% Input:
%           dh: [?x4]as the order: d, theta, a, alpha
% Output:
%           T: [4x4] Transformation matrix {i}^T_{j}
% 
% Source:     
%       Wiki: https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters


function T = T_dh(dh)

assert(size(dh,2)== 4, 'func T_dh error: dimension of mdh is incorrect!')

num_frame = size(dh,1);
T = eye(4);

for i = 1:num_frame
    
    d = dh(i,1);
    theta = dh(i,2);
    a = dh(i,3);
    alpha = dh(i,4);
    
    T_tmp = [cos(theta), -sin(theta)*cos(alpha) ,    sin(theta)*sin(alpha),     a*cos(theta);
        sin(theta),      cos(theta)*cos(alpha),      -cos(theta)*sin(alpha),    a*sin(theta);
        0 ,             sin(alpha),                  cos(alpha),                d;
        0,                     0,                      0,           1];
    T = T*T_tmp;
end
end

