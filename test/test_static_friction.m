% static friction model test
% including stiction, stribeck, coulumb, viscous friction  



clear all
close all
clc

%% initialize parameters

parameter_struct = static_friction_par_init();
f_e = 1;
f_n = 20;
v = [-10:0.01:10]';


%% friction computation
f_static = [];
for i = 1:length(v)
    f_static(i) = static_friction_f(v(i),f_n,f_e, parameter_struct);
end

%% plot figure
figure(1)

plot(v,f_static)
grid on
