function yout =  heun(fun,dt,t0,y0)
% second order explicit trapezoid
% 0  | 0   0
% 1  | 1   0
% __________
%    |0.5 0.5


k1 = fun(t0,y0);
% size(f1)
% size(y0)
k2 = fun(t0+dt,y0+(dt)*k1);


yout = y0 + dt/2 *(k1 + k2);