function yout =  midpoint(fun,dt,t0,y0)
% 0  | 0   0
% 0.5| 0.5 0
% __________
%    | 0   1


k1 = fun(t0,y0);
% size(f1)
% size(y0)
k2 = fun(t0+dt/2,y0+(dt/2)*k1);


yout = y0 + dt*k2;