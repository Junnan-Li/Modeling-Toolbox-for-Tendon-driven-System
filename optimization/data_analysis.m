% Data analysis and map visualization 




% Junnan Li, junnan.li@tum.de, MIRMI, 06.2023

%% load work space

load ./optimization/results/workspace_0506_x_1mm_downward.mat

%% Data Analysis

% force index
mani_force_polytope = force_index;

% acceleration polytope volume
mani_acc_v = vol_sample_acc;
acc_v_max = max(mani_acc_v);
acc_v_min = min(mani_acc_v);
acc_v_index_normalized = (mani_acc_v - acc_v_min)/(acc_v_max-acc_v_min);


% acceleration index
mani_acc_r = r_sample_acc;
acc_r_max = max(r_sample_acc);
acc_r_min = min(r_sample_acc);
acc_r_index_normalized = (mani_acc_r - acc_r_min)/(acc_r_max-acc_r_min);

% joint angle index
q_limit_low = q_limit(:,1);
q_limit_upper = q_limit(:,2);
mani_q = q_sample_r*180/pi;
mani_q_normalized = (mani_q - q_limit_low')./(q_limit_upper'-q_limit_low');

% use sin wave for joint limits
joint_limits_index_sin = sin(mani_q_normalized*2*pi-pi/2)+2;
joint_limits_index = (joint_limits_index_sin(:,1).*joint_limits_index_sin(:,2).*...
    joint_limits_index_sin(:,3).*joint_limits_index_sin(:,4));

joint_limits_index_normalized = (joint_limits_index-min(joint_limits_index))/...
    (max(joint_limits_index)-min(joint_limits_index));


% metric value
% metric_value = zeros(size(mani_force_polytope));
metric_normalized = zeros(size(mani_force_polytope,1),1);


% Weight vector of force index

W = [1,0,0,0,0]';
% W = [1,0.2,0.2,0.2,0.2]';

% mani_index_plot_j = mani_force_polytope;% *W;

% normalize the force index of one direction
index_max_force_pol = max(mani_force_polytope);
index_min_force_pol = min(mani_force_polytope);
%

force_pol_index_normalized_all = (mani_force_polytope - index_min_force_pol)./(index_max_force_pol-index_min_force_pol);
force_pol_index_normalized = force_pol_index_normalized_all * W;


metric_j = joint_limits_index_normalized .* acc_r_index_normalized.* ...
    acc_v_index_normalized.* force_pol_index_normalized;

% metric_j = joint_limits_index_normalized .* acc_r_index_normalized .* force_pol_index_normalized;


% normalized
metric_j_max = max(metric_j);
metric_j_min = min(metric_j);
metric_j_normalized = (metric_j-metric_j_min)/(metric_j_max-metric_j_min);

metric_normalized = metric_j_normalized;






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


for subplot_i = 1:5

%     Pos_Car_occupied = zeros(dim_Car+2);
    subplot(2,3,subplot_i)
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
            metric_term = acc_r_index_normalized;
            title_plot = 'Acc. radius';
        case 3
            metric_term = acc_v_index_normalized;
            title_plot = 'Acc. volume';
        case 4
            metric_term = force_pol_index_normalized;
            title_plot = 'Force index';
        case 5
            metric_term = metric_j_normalized;
            title_plot = 'metric index';
        otherwise
            disp('other value')
    end

    
    index_vec = find(states_sample(:) == 1);
    C = [metric_term(index_vec) zeros(size(metric_term(index_vec))) -metric_term(index_vec)] + [0 0 1];
    
   
    % origin finger
    hs = scatter3(pos_sample_r(index_vec,1),pos_sample_r(index_vec,2),pos_sample_r(index_vec,3),10,...
        C,'square','MarkerEdgeAlpha',0.3,'MarkerFaceAlpha',.2);
    
    hold on
    [~,I] = max(metric_term);
    q_i = q_sample_r(I,:)';
    finger_analysis.update_finger(q_i);
    finger_analysis.print_finger('k');
    
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

subplot(2,3,6)
for i = 1%:5
    finger_analysis.print_finger('k');
    quiver3(pos_sample_r(I,1),pos_sample_r(I,2),pos_sample_r(I,3),...
        direction_vec(i,1)/50,direction_vec(i,2)/50,direction_vec(i,3)/50,'Color','r','LineWidth',5,'MaxHeadSize',5);
    axis equal
%     hold on
%     finger_analysis_rot.print_finger('k');
%     quiver3(pos_sample_modified(I,1),pos_sample_modified(I,2),pos_sample_modified(I,3),...
%         -direction_vec(i,1)/50,-direction_vec(i,2)/50,-direction_vec(i,3)/50,'Color','r','LineWidth',5,'MaxHeadSize',5);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title('Force direction (neg. Z-axis)')
end

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