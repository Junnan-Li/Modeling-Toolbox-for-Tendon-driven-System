% sarturate the vector values with the minimal and maxiaml limits
% 

function vec_sat = vec_saturation(vec, range)

num = length(vec);
assert(all(size(range)== [num,2]), '[vec_saturation]: dimension of input is incorrect!')
vec_sat = vec;

if ~isempty(find(vec_sat<range(:,1)))
    vec_sat(vec_sat<range(:,1)) = range(vec_sat<range(:,1),1);
end
if ~isempty(find(vec_sat>range(:,2)))
    vec_sat(vec_sat>range(:,2)) = range(vec_sat>range(:,2),2);
end
end