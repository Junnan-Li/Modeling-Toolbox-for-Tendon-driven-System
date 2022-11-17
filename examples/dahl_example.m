%  Dahl model



clear all
close all
clc


dt = 1e-3;
sigma = 10;
F_c = 6;
v = 4;


T = [];
T(1) = 0;

f = [];
f(1) = 1;

for i = 1:10000
    
    t = T(i);
    
    if i > 5000
        v = v + 0.1;
    end
    
    % runge kutta 4 solver
    f_k = f(i);
    f_k1 = rungekutta4(@(t,f) dahl_friction_dfdt(t,f,v,sigma,F_c), dt,t,f_k);
    f(i+1) = f_k1;
    
    T(i+1) = t + dt;
end

figure(1)

plot(T,f)

