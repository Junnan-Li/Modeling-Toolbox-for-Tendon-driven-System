% use Transformation matrix on a 3x1 dimension position vector
% 

function p_new = T_p31(T, p)

assert(all(size(T)== [4,4]), '[T_p]: dimension of T is incorrect!')
assert(length(p) == 3, '[T_p]: dimension of p is incorrect!')

p_41 = [reshape(p,3,1);1];

p_new_41 = T*p_41;
p_new = p_new_41(1:3);

end