% generation figures for paper




% Junnan Li, junnan.li@tum.de, MIRMI, 06.2023

%% load work space
clear all
close all 
clc

load ./optimization/results/workspace_1406_x_4mm_30_downwards_index.mat



%% force direction definition of index finger
% directions: 5
% Ae_all [2*5x3], the vectors that are orthogonal to the direction vectors
SQ2 = sqrt(2)/2;
Ae_all = [1 0 0;...
    0 1 0;... % 1: -z
    SQ2 0 SQ2;...
    0 1 0;... % 2: +x-z
    SQ2 0 -SQ2;...
    0 1 0;...% 3: -x-z
    1 0 0;...
    0 SQ2 SQ2;...% 4: +y-z
    1 0 0;...
    0 SQ2 -SQ2;...% 5: -y-z
    0 0 1;...
    0 1 0;... % 6: +x
    0 0 1;...
    0 -1 0;... % 7: -x
    0 0 1;...
    -1 0 0;... % 8: +y
    0 0 1;...
    1 0 0;... % 9: -y
    ];


% 5 direction vectors
direction_vec = [0 0 -1;...
    SQ2 0 -SQ2;...
    -SQ2 0 -SQ2;...
    0 SQ2 -SQ2;...
    0 -SQ2 -SQ2;...
    1 0 0;...
    -1 0 0;...
    0 1 0;...
    0 -1 0];



%% Data Analysis
% cell: 
%   1 pos |2 q 2x4 |3 forceindex 5 |4 vol_force |5 acc. r |6 acc.vol |7 conJ |
%   8 conJM |9 status 
% 
% states: 1: success with nullspace solution
%         5: success without nullspace solution
%       10: no ik solution
%       1e2: Jacobian deficient
%       1e3: acceleration polytope volume too small
%       4e3: acceleration polytope volume too large
%       1e4: unable to get acceleration volume
%       1e5: unable to get force volume


% force_index = zeros(n_sample,10);
% vol_sample_acc = zeros(n_sample,2);
% r_sample_acc = zeros(n_sample,2);
% q_sample_r = zeros(n_sample,8);
% pos_sample_r = zeros(n_sample,3);
% status = zeros(n_sample,2);

q_sample_1 = result{2}{1};
q_sample_2 = result{2}{2};
force_index_1 = result{3}{1};
force_index_2 = result{3}{2};
f_vol_1 = result{4}{1};
f_vol_2 = result{4}{2};
acc_r_1 = result{5}{1};
acc_r_2 = result{5}{2};
pos_sample_r = result{1};
status = result{9};
%%

% i = 64;
% index = 6207;

M_coupling = [0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01, -0.01;...
              0.01, 0.01, 0.01,   0,   -0.01, -0.01, -0.01, -0;...
              0.01, 0.01, 0,      0,   -0.01, -0.01, 0 ,     0;...
              0.01, 0,    0,      0,   -0.01, 0 ,    0,     0];

index = find(abs(pos_sample_r(:,1)-0.05) < 0.0015 & abs(pos_sample_r(:,2)-0.0) < 0.0015& ...
    abs(pos_sample_r(:,3)- (-0.055)) < 0.0015 );

% index = find(abs(pos_sample_r(:,1)-0.07) < 0.0015 & abs(pos_sample_r(:,2)-0.0) < 0.0015& ...
%     abs(pos_sample_r(:,3)- (-0.025)) < 0.0015 );

pos_sample_r(index,:);
status(index);
q_i = result{2}{1}(index,:)';
% q_i = [0;0.5;0.6;0.4];

finger_analysis.update_finger(q_i);



% calculate the index
% get Jacobian
J_index = finger_analysis.Jacobian_analytic_b_end;
% get reduced Jacobian only with translational terms
J_index_red = finger_analysis.w_R_base * J_index(1:3,:);

rank(J_index_red);

% calculate the Mass matrix
[~,M_r,~,~] = finger_analysis.fordyn_ne_w_end(q_i, zeros(4,1), zeros(4,1), zeros(6,6));
% calculate the Cartesian acceleration polytope
P_acc = J_index_red * inv(M_r) * P_tau;
P_acc2 = J_index_red * P_tau;

% check condition number of J*M-1
cond(J_index_red * inv(M_r));
cond(J_index_red);
% calculate the volume of the acceleration polytope
try
%     P_acc.volume;
    largest_minimum_radius_P_input(P_acc, [0,0,0]');
end


% calculate the force polytope
P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration

% calculate the force polytope
force_index_i = zeros(9,1);
force_V_i = zeros(9,3);

for mi = 1:9 % direction index
    % reduce the dimension from 3 to 1 along the direction
    % vector
    p_tmp_force = Polyhedron('A', P_ee.A, 'b', P_ee.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
    Vertex_f_i = p_tmp_force.V;
    if isempty(Vertex_f_i)
        force_index_i(mi) = 0;
    elseif direction_downward
        force_index_i(mi) = sqrt(Vertex_f_i(1,:)*Vertex_f_i(1,:)'); % vextex of the 1-dimension polytope
    else
        force_index_i(mi) = sqrt(Vertex_f_i(2,:)*Vertex_f_i(2,:)'); % vextex of the 1-dimension polytope
    end
    force_V_i(mi,:) = force_index_i(mi) * direction_vec(mi,:);
end

% return
%% plot generation


h = figure(20);
set(0,'defaultfigurecolor','w')
set(groot,'defaulttextinterpreter','none');
set(h,'units','normalized','outerposition',[0 0 1 1])
set(h, 'Units', 'centimeters')
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)]);%
set(h, 'Units', 'centimeters')
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

% subplot(1,2,1)
% finger_analysis.print_finger
% axis equal
% 
% subplot(1,2,2)

P_ee_half = Polyhedron('A', [P_ee.A;0,0,1], 'b', [P_ee.b;0]);
P_ee_half.plot('Color','c','alpha',0.3)
hold on 
P_app = Polyhedron('V', force_V_i);
P_app.plot('Color','r','alpha',1)
hold on
plot3(force_V_i(:,1),force_V_i(:,2),force_V_i(:,3),'.','Color','k','MarkerSize',100);
grid off

xlabel('x')
ylabel('y')
zlabel('z')
axis equal


fprintf('Volume ratio: %f / %f = %f \n', P_app.volume, P_ee_half.volume, P_app.volume/P_ee_half.volume)

%%  tendon space - torque - force polytope



