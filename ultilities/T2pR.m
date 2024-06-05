% Transformation matrix to position vector and Rotation matrix
% 

function [p, R] = T2pR(T)

assert(all(size(T)== [4,4]), '[T2pR]: dimension of T is incorrect!')

p = T(1:3,4);
R = T(1:3,1:3);

end