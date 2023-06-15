% test joint limits penalization functions



% clear all
% clc,


%% define joint limits 
q_limit = [0,90;0,80];
q = q_limit(1,1):.2:q_limit(1,2);

[q_1,q_2] = ndgrid(q_limit(1,1):1:q_limit(1,2),...
    q_limit(2,1):1:q_limit(2,2));
%% 

% [1] T. F. Chan and R. V Dubey, "A Weighted Least-Norm Solution Based Scheme for Avoiding Joint
% Limits for Redundant Joint Manipulators,” IEEE Trans. Robot. Autom., vol. 1, no. 2, 1995.
% [2] N. Vahrenkamp and T. Asfour, "Representing the robot[s workspace through constrained manipulability
% analysis," Auton. Robots, vol. 38, no. 1, pp. 17–30, 2015, doi: 10.1007/s10514-014-9394-z.

% penalty_1 = @(x,q_limits_low,q_limits_high) 1./(sqrt(1+abs(Hdot(q,q_limits_low,q_limits_high))));
penal_jl_i = penalty_joint_limits([q_1(:),q_2(:)]', q_limit,2)';
penal_jl = penal_jl_i(:,1).*penal_jl_i(:,2);

C = [1 0 0] .* penal_jl ;

figure(1)

hs = scatter3(q_1(:),q_2(:),penal_jl,10,C);
% plot3(q_1(:),q_2(:),penal_jl,'.','Color',[1 0 0].*penal_jl)
% grid on
% % axis equal

figure(2)
hs = plot(q_1(:),penal_jl_i(:,1));
xlabel('deg')
ylabel('penalty value')



