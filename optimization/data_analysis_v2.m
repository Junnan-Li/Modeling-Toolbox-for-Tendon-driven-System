% Data analysis and map visualization 




% Junnan Li, junnan.li@tum.de, MIRMI, 06.2023

%% load work space
clear all
close all 
clc

load ./optimization/results/workspace_1406_x_4mm_30_downwards_index.mat



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

penal_acc_r = zeros(n_sample,2);
joint_limits_index_normalized = zeros(n_sample,2);
force_pol_index_normalized = zeros(n_sample,2);
metric_normalized = zeros(n_sample,2);

for i_ns = 1:2

    % acceleration index
    mani_acc_r = result{5}{i_ns};
    acc_r_max = max(mani_acc_r);
    acc_r_min = min(mani_acc_r);
    acc_r_index_normalized = (mani_acc_r - acc_r_min)/(acc_r_max-acc_r_min);
    penal_acc_r(:,i_ns) = penalty_acc_r(acc_r_index_normalized, 1);

    % joint angle index
    %%%%%%%
    % q_limit_low = q_limit(:,1);
    % q_limit_upper = q_limit(:,2);
    % mani_q = q_sample_r*180/pi;
    % mani_q_normalized = (mani_q - q_limit_low')./(q_limit_upper'-q_limit_low');
    %
    % % use sin wave for joint limits
    % joint_limits_index_sin = sin(mani_q_normalized*2*pi-pi/2)+2;
    % joint_limits_index = (joint_limits_index_sin(:,1).*joint_limits_index_sin(:,2).*...
    %     joint_limits_index_sin(:,3).*joint_limits_index_sin(:,4));
    %
    % joint_limits_index_normalized = (joint_limits_index-min(joint_limits_index))/...
    %     (max(joint_limits_index)-min(joint_limits_index));
    %%%%%%%
    q_sample_r = result{2}{i_ns};
    mani_q = q_sample_r*180/pi;
    joint_limits_index_i = penalty_joint_limits(mani_q', q_limit,1)';
    joint_limits_index = joint_limits_index_i(:,1).* joint_limits_index_i(:,2) ...
        .* joint_limits_index_i(:,3) .* joint_limits_index_i(:,4);
    joint_limits_index_normalized(:,i_ns) = (joint_limits_index-min(joint_limits_index))/...
        (max(joint_limits_index)-min(joint_limits_index));

    % metric value
    % metric_value = zeros(size(mani_force_polytope));
    
    mani_force_polytope = result{3}{i_ns};

    % Weight vector of force index

    W = [1,0,0,0,0]'/1;
%     W = [1,0.1,0.1,0.1,0.1]'/1.4;
    % normalize the force index of one direction
    
    %
    max(mani_force_polytope);


    
    force_pol_index_all = mani_force_polytope * W;
    
    index_max_force_pol = max(force_pol_index_all);
    index_min_force_pol = min(force_pol_index_all);

    force_pol_index_normalized_all = (force_pol_index_all - index_min_force_pol)./(index_max_force_pol-index_min_force_pol);
    force_pol_index_normalized(:,i_ns) = force_pol_index_normalized_all ;


    % metric_j = joint_limits_index_normalized .* acc_r_index_normalized.* ...
    %     acc_v_index_normalized.* force_pol_index_normalized;

    metric_j = joint_limits_index_normalized .* penal_acc_r .* (force_pol_index_normalized);


    % normalized
    metric_j_max = max(metric_j);
    metric_j_min = min(metric_j);
    metric_j_normalized = (metric_j-metric_j_min)/(metric_j_max-metric_j_min);

    metric_normalized(:,i_ns) = metric_j_normalized;

end 



%% results Cartesian space
% C_min = min(pos_sample_r);
% C_max = max(pos_sample_r);
% voxel_length = 0.001;
% dim_Car = floor((C_max-C_min)/voxel_length); % 0.01m
% 
% voxel_min = floor(C_min/voxel_length);
% 
% Pos_Car_occupied = zeros(dim_Car+2);


h = figure(15);
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


for subplot_i = 1:4

%     Pos_Car_occupied = zeros(dim_Car+2);
    subplot(1,4,subplot_i)
%     figure(22)

    finger_analysis.update_finger([0,0,0,0]')
%     finger_analysis.print_finger('g')
    finger_analysis.update_finger([0;finger_analysis.limits_q(2:4,2)])
%     finger_analysis.print_finger('g')
    
    switch subplot_i
        case 1
            metric_term = joint_limits_index_normalized;
            title_plot = 'Joint limits';
        case 2
            metric_term = penal_acc_r;
            title_plot = 'Acc. radius';
        case 3
            metric_term = force_pol_index_normalized;
            title_plot = 'Force index';
        case 4
            metric_term = metric_normalized;
            title_plot = 'metric index';
        otherwise
            disp('other value')
    end
    
    [metric_term,I_metric_term] = max(metric_term,[],2);

    index_vec = find(status(:) >= 1 ); %  & abs(pos_sample_r(:,2)-0.0) < 0.0015 
    C = [metric_term(index_vec) zeros(size(metric_term(index_vec))) -metric_term(index_vec)] + [0 0 1];

%     index_vec = find(status(:) >= 1 ); % & abs(pos_sample_r(:,1)-0.056) < 0.0015  
%     C = [metric_term(index_vec) zeros(length(index_vec),1) -metric_term(index_vec)] + [0 0 1];

   
    % origin finger
    hs = scatter3(pos_sample_r(index_vec,1),pos_sample_r(index_vec,2),pos_sample_r(index_vec,3),30,...
        C,'square','filled','MarkerEdgeAlpha',.3,'MarkerFaceAlpha',.3);
    
    hold on
    [~,I] = max(metric_term);
    q_i = result{2}{I_metric_term(I)}(I,:)';
    finger_analysis.update_finger(q_i);
    finger_analysis.print_finger('k');

    fprintf("       JL,     Acc.R.,     F.I.,     M: \n")
    fprintf("Index: %f, %f, %f, %f \n", joint_limits_index_normalized(I), acc_r_index_normalized(I),...
                    force_pol_index_normalized(I), metric_j_normalized(I))   

    % rotated finger
%     R = euler2R_XYZ([pi,0,0]);
%     T = [0,0,-0.1];
%     pos_sample_modified = pos_sample_r * R' + T;
%     hs = scatter3(pos_sample_modified(index_vec,1),pos_sample_modified(index_vec,2),pos_sample_modified(index_vec,3),10,...
%         C,'square','MarkerEdgeAlpha',0.6,'MarkerFaceAlpha',.2);
%     
%     hold on
%     [~,I] = max(metric_term);
%     q_i = q_sample_r(I,:)';
%     finger_analysis_rot = copy(finger_analysis);
%     finger_analysis_rot.w_p_base = T';
%     finger_analysis_rot.w_R_base = R*finger_analysis_rot.w_R_base;
%     finger_analysis_rot.update_finger(q_i);
%     finger_analysis_rot.print_finger('k');
%     
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title(title_plot)

end



% subplot(2,3,6)
% for i = 1%:5
%     finger_analysis.print_finger('k');
%     quiver3(pos_sample_r(I,1),pos_sample_r(I,2),pos_sample_r(I,3),...
%         direction_vec(i,1)/50,direction_vec(i,2)/50,direction_vec(i,3)/50,'Color','r','LineWidth',5,'MaxHeadSize',5);
%     axis equal
% %     hold on
% %     finger_analysis_rot.print_finger('k');
% %     quiver3(pos_sample_modified(I,1),pos_sample_modified(I,2),pos_sample_modified(I,3),...
% %         -direction_vec(i,1)/50,-direction_vec(i,2)/50,-direction_vec(i,3)/50,'Color','r','LineWidth',5,'MaxHeadSize',5);
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
%     title('Force direction (neg. Z-axis)')
% end

return

%% plot result for two finger
metric_term = metric_j_normalized;
h = figure(16);
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


subplot(1,2,1)

index_vec = find(states_sample(:) == 1);
C = [metric_term(index_vec) zeros(size(metric_term(index_vec))) -metric_term(index_vec)] + [0 0 1];

% origin finger
hs = scatter3(pos_sample_r(index_vec,1),pos_sample_r(index_vec,2),pos_sample_r(index_vec,3),30,...
    C,'filled','MarkerEdgeAlpha',0.2,'MarkerFaceAlpha',.2);

hold on
[~,I] = max(metric_term);
q_i = q_sample_r(I,:)';
finger_analysis.update_finger(q_i);
finger_analysis.print_finger('k');
% rotated finger
R = euler2R_XYZ([pi,0,0]);
T = [0,0,-0.1];
pos_sample_modified = pos_sample_r * R' + T;
hs = scatter3(pos_sample_modified(index_vec,1),pos_sample_modified(index_vec,2),pos_sample_modified(index_vec,3),30,...
    C,'filled','MarkerEdgeAlpha',0.2,'MarkerFaceAlpha',.2);
hold on
finger_analysis_rot = copy(finger_analysis);
finger_analysis_rot.w_p_base = T';
finger_analysis_rot.w_R_base = R*finger_analysis_rot.w_R_base;
finger_analysis_rot.update_finger(q_i);
finger_analysis_rot.print_finger('k');

subplot(1,2,2)

index_vec = find(states_sample(:) == 1 & abs(pos_sample_r(:,2)) < 0.001);
C = [metric_term(index_vec) zeros(size(metric_term(index_vec))) -metric_term(index_vec)] + [0 0 1];

% origin finger
hs = scatter3(pos_sample_r(index_vec,1),pos_sample_r(index_vec,2),pos_sample_r(index_vec,3),120,...
    C,'square', 'filled','MarkerEdgeAlpha',0.6,'MarkerFaceAlpha',.6);
hold on
[~,I] = max(metric_term(index_vec));
q_i = q_sample_r(index_vec(I),:)';
finger_analysis.update_finger(q_i);
finger_analysis.print_finger('k');

% rotated finger
R = euler2R_XYZ([pi,0,0]);
T = [0,0,-0.1];
pos_sample_modified = pos_sample_r * R' + T;
hs = scatter3(pos_sample_modified(index_vec,1),pos_sample_modified(index_vec,2),pos_sample_modified(index_vec,3),120,...
    C,'square', 'filled','MarkerEdgeAlpha',0.6,'MarkerFaceAlpha',.6);

hold on
% [~,I] = max(metric_term);
% q_i = q_sample_r(I,:)';
finger_analysis_rot = copy(finger_analysis);
finger_analysis_rot.w_p_base = T';
finger_analysis_rot.w_R_base = R*finger_analysis_rot.w_R_base;
finger_analysis_rot.update_finger(q_i);
finger_analysis_rot.print_finger('k');

axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('metric map of two-finger hand')


return

%% plot result for multi finger
metric_term = metric_j_normalized;

origin_fingers = [0,0,0,0;...
                0,0.08,-0.08,0.04;...
                0,-0.08,-0.08,0];
orientation_fingers = [0,-120,120,-30]*pi/180; % along x axis

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


subplot(1,2,1)

index_vec = find(states_sample(:) == 1);
C = [metric_term(index_vec) zeros(size(metric_term(index_vec))) -metric_term(index_vec)] + [0 0 1];

for i = 1:3% 4
    
    % rotated finger
    R = euler2R_XYZ([orientation_fingers(i),0,0]);
    T = origin_fingers(:,i)';
    pos_sample_modified = pos_sample_r * R' + T;
    hs = scatter3(pos_sample_modified(index_vec,1),pos_sample_modified(index_vec,2),pos_sample_modified(index_vec,3),15,...
        C,'filled','MarkerEdgeAlpha',0,'MarkerFaceAlpha',0.1);
    hold on
    hold on
    [~,I] = max(metric_term);
    q_i = q_sample_r(I,:)';

    finger_analysis_rot = copy(finger_analysis);
    finger_analysis_rot.w_p_base = T';
    finger_analysis_rot.w_R_base = R*finger_analysis_rot.w_R_base;
    finger_analysis_rot.update_finger(q_i);
    finger_analysis_rot.print_finger('k');
    
end
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('metric map of four-finger hand')


subplot(1,2,2)

index_vec = find(states_sample(:) == 1  & abs(pos_sample_r(:,1)-0.06) < 0.001);
C = [metric_term(index_vec) zeros(size(metric_term(index_vec))) -metric_term(index_vec)] + [0 0 1];

for i = 1:3%4
    
    % rotated finger
    R = euler2R_XYZ([orientation_fingers(i),0,0]);
    T = origin_fingers(:,i)';
    pos_sample_modified = pos_sample_r * R' + T;
    hs = scatter3(pos_sample_modified(index_vec,1),pos_sample_modified(index_vec,2),pos_sample_modified(index_vec,3),30,...
        C,'square','filled','MarkerEdgeAlpha',0.6,'MarkerFaceAlpha',.6);
    hold on
    hold on
    [~,I] = max(metric_term);
    q_i = q_sample_r(I,:)';

    finger_analysis_rot = copy(finger_analysis);
    finger_analysis_rot.w_p_base = T';
    finger_analysis_rot.w_R_base = R*finger_analysis_rot.w_R_base;
    finger_analysis_rot.update_finger(q_i);
    finger_analysis_rot.print_finger('k');
    
end
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('metric map of four-finger hand')


return

%% debud

i = 64;
% index = 6207;

index = find(abs(pos_sample_r(:,1)-0.077) < 0.0015 & abs(pos_sample_r(:,2)-0.0) < 0.0015& ...
    abs(pos_sample_r(:,3)- (-0.016)) < 0.0015 )

pos_sample_r(index,:)
status(index)
q_i = result{2}{1}(index,:)';
% q_i = [0;0.5;0.6;0.4];

finger_analysis.update_finger(q_i);
figure(100)
subplot(1,3,1)
finger_analysis.print_finger
axis equal
% calculate the index
% get Jacobian
J_index = finger_analysis.Jacobian_analytic_b_end;
% get reduced Jacobian only with translational terms
J_index_red = finger_analysis.w_R_base * J_index(1:3,:);

rank(J_index_red)

% calculate the Mass matrix
[~,M_r,~,~] = finger_analysis.fordyn_ne_w_end(q_i, zeros(4,1), zeros(4,1), zeros(6,6));
% calculate the Cartesian acceleration polytope
P_acc = J_index_red * inv(M_r) * P_tau;
P_acc2 = J_index_red * P_tau;
subplot(1,3,2)
P_acc.plot
axis equal
% check condition number of J*M-1
cond(J_index_red * inv(M_r));
cond(J_index_red);
% calculate the volume of the acceleration polytope
try
%     P_acc.volume;
    largest_minimum_radius_P_input(P_acc, [0,0,0]')
end


% calculate the force polytope
P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
subplot(1,3,3)
P_ee.plot
axis equal
% calculate the force polytope
for mi = 1:5 % direction index
    % reduce the dimension from 3 to 1 along the direction
    % vector
    p_tmp_force = Polyhedron('A', P_ee.A, 'b', P_ee.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
    Vertex_f_i = p_tmp_force.V;
    if isempty(Vertex_f_i)
        max_pos_f_Ver_i = 0;
    elseif direction_downward
        max_pos_f_Ver_i = -min(Vertex_f_i(:,3)); % vextex of the 1-dimension polytope
    else
        max_pos_f_Ver_i = max(Vertex_f_i(:,3)); % vextex of the 1-dimension polytope
    end
end

