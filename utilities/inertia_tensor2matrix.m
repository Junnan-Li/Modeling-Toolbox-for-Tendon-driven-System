function I = inertia_tensor2matrix(inertia_tensor)
%
assert(length(inertia_tensor) == 6, 'Function inertia_tensor2matrix error: input dimension is incorrect');

I_xx = inertia_tensor(1);
I_yy = inertia_tensor(2);
I_zz = inertia_tensor(3);
% I_xy = inertia_tensor(4);
% I_xz = inertia_tensor(5);
% I_yz = inertia_tensor(6);

I_yz = inertia_tensor(4);
I_xz = inertia_tensor(5);
I_xy = inertia_tensor(6);

I = [I_xx, I_xy, I_xz;...
     I_xy   , I_yy, I_yz;...
     I_yz   , I_xz   , I_zz];

end