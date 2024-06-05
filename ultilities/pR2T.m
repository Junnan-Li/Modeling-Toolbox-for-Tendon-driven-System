%  position vector and Rotation matrix to Transformation matrix
% 



function T = pR2T(p,R)

assert(all(size(p)== [3,1]), '[pR2T]: dimension of p is incorrect!')
assert(all(size(R)== [3,3]), '[pR2T]: dimension of R is incorrect!')

T = [R,p;0 0 0 1];

end