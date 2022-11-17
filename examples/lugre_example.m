%  Lugre friction model test



clear all
close all
clc


% time step
dt = 1e-3;

% parameters of the lugre model
sigma0 = 1;
sigma1 = 0.002;
sigma2 = 0.02;

F_c = 0.3;
F_s = 0.26;
v = 0;
v_s = 0.9;
alpha = 2;


% time vector
T = [];
T(1) = 0;

% states vector
z = [];
z(1) = 1;

% friction force & velocity vector
f = []; % 
f(1,:) = [0,v];% initial force

for i = 1:10000
    
    t = T(i);
    z = z(i);
    if i > 5000
        v = v + 0.02;
%         v = sin((i-5000)/1000*pi);
    end
    dz_k = lugre_friction_dzdt(t,z,v,sigma0,F_c,F_s,v_s,alpha);
    f(i,1) = lugre_friction_f(sigma0, sigma1, sigma2, z, dz_k, v);
    f(i,2) = v;
    % runge kutta 4 solver
    z_k1 = rungekutta4(@(t,z) lugre_friction_dzdt(t,z,v,sigma0,F_c,F_s,v_s,alpha), dt,t,z);
    z(i+1) = z_k1;
    
    T(i+1) = t + dt;
end

figure(1)

plot(f(:,2),f(:,1))
figure(2)
subplot(2,1,1)
plot(T(1:i),f(:,1))
subplot(2,1,2)
plot(T(1:i),f(:,2))

