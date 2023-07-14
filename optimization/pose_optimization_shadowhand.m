









clear all
% close all 
clc

% load ./optimization/results/workspace_3006_x_3mm_30_shadow.mat % shadow hand data with changed d_0 
load ./optimization/results/workspace_3006_x_3mm_30_shadow_fixed.mat % shadow hand data with changed d_0 
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

Metric = struct();

for i = 1:3
    
    if i == 1
        result = result_all.fingers;
        finger_analysis = finger_index;
    elseif i == 2
        result = result_all.thumb;
        finger_analysis = finger_thumb;
    elseif i == 3
        result = result_all.little;
        finger_analysis = finger_little;
    end
    q_limit = finger_analysis.limits_q(:,1:2);
    
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
    
%     force_index_1(force_index_1(:)>1e3)=0;
%     force_index_2(force_index_2(:)>1e3)=0;
    %
    n_sample = size(result{1},1);
    penal_acc_r = zeros(n_sample,2);
    acc_r_index_normalized = zeros(n_sample,2);
%     joint_limits_index_i = zeros(n_sample,1);
    joint_limits_index_normalized = zeros(n_sample,2);
    force_pol_index_normalized = zeros(n_sample,2);
    metric_normalized = zeros(n_sample,2);
    
    for i_ns = 1:2
        
        % acceleration index
        mani_acc_r = result{5}{i_ns};
        %     acc_r_max = max(mani_acc_r);
        %     acc_r_min = min(mani_acc_r);
        %     acc_r_index_normalized = (mani_acc_r - acc_r_min)/(acc_r_max-acc_r_min);
        
        acc_r_index_normalized(:,i_ns) = normalize_vector(mani_acc_r, 1);
        penal_acc_r(:,i_ns) = penalty_acc_r(acc_r_index_normalized(:,i_ns), 1);
        
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
        q_limit_deg = q_limit*180/pi;
        joint_limits_index_i = penalty_joint_limits(mani_q', q_limit_deg,4)';
        joint_limits_index = ones(size(joint_limits_index_i(:,1)));
        for jl_index = 1:finger_analysis.nja
            joint_limits_index = joint_limits_index .* joint_limits_index_i(:,jl_index);
        end
        joint_limits_index_normalized(:,i_ns) = (joint_limits_index-min(joint_limits_index))/...
            (max(joint_limits_index)-min(joint_limits_index));
        
        
        % metric value
        % metric_value = zeros(size(mani_force_polytope));
        
        mani_force_polytope = result{3}{i_ns};
%         mani_force_polytope(mani_force_polytope(:)>800 )=0;
        % Weight vector of force index
        %
        % W = [0,0,0,0.2,0,0,0,0,0]'/0.2;
        % W = [1,0,0,0,0,0,0,0,0]'/1;
%         W = [0.4,0.2,0.2,0.2,0.2,0,0,0,0]'/1.2;
        W = [0.2,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]'/1;
        % normalize the force index of one direction
        
        %
        max(mani_force_polytope);
        
        
        
        force_pol_index_all = mani_force_polytope * W;
        
        index_max_force_pol = max(force_pol_index_all);
        index_min_force_pol = min(force_pol_index_all);
        
        force_pol_index_normalized_all = (force_pol_index_all - index_min_force_pol)./(index_max_force_pol-index_min_force_pol);
        force_pol_index_normalized(:,i_ns) = force_pol_index_normalized_all ;
        
        
        
        
    end
    % metric_j = joint_limits_index_normalized .* acc_r_index_normalized.* ...
    %     acc_v_index_normalized.* force_pol_index_normalized;
    
    metric_j = joint_limits_index_normalized .* penal_acc_r .* (force_pol_index_normalized);
    
    % normalized
    metric_j_max = max(metric_j);
    metric_j_min = min(metric_j);
    metric_normalized = (metric_j - metric_j_min) ./ (metric_j_max-metric_j_min);
    
    if i == 1
        Metric.fingers.metric_n = metric_normalized;
        Metric.fingers.fi_n = force_pol_index_normalized;
        Metric.fingers.jl_n = joint_limits_index_normalized;
        Metric.fingers.acc_n = penal_acc_r;
        Metric.fingers.pos = pos_sample_r;
    elseif i == 2
        Metric.thumb.metric_n = metric_normalized;
        Metric.thumb.fi_n = force_pol_index_normalized;
        Metric.thumb.jl_n = joint_limits_index_normalized;
        Metric.thumb.acc_n = penal_acc_r;
        Metric.thumb.pos = pos_sample_r;
    elseif i == 3
        Metric.little.metric_n = metric_normalized;
        Metric.little.fi_n = force_pol_index_normalized;
        Metric.little.jl_n = joint_limits_index_normalized;
        Metric.little.acc_n = penal_acc_r;
        Metric.little.pos = pos_sample_r;
    end
    
end

Metric.middle = Metric.fingers;
Metric.middle.pos = Metric.middle.pos + [0.133,0.011,0] - [0.129,0.034,0];


%% object setting
% contact points: -pi/2, pi*2/3, 

r_object = 0.025; % 4cm sphere

%% 
fitnessfcn_fcn = @(X_object)fitness_metric_three_fingers_grasp(X_object,r_object,Metric);

options = optimoptions('particleswarm','MaxIterations',20,'Display','iter','SwarmSize',1000);

var_limits = [0,0.2;-0.05,0.1;-0.06,0.1;-pi/2,pi/2;-pi/2,pi/2;-pi/2,pi/2];

fprintf('optimization start! \n')

[x,fval,exitflag,output] = particleswarm(fitnessfcn_fcn,6,var_limits(:,1),var_limits(:,2),options);

fval = fitness_metric_three_fingers_grasp(x,r_object,Metric);

return
%%
%   result 1: -0.0796
%   result 2: -0.1392
%   result 3: -0.1405
%   result 4: -0.2821
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

p_object = x(1:3);
phi_object = x(4:6);
p_c_thumb = r_object * euler2R_XYZ(phi_object)*euler2R_XYZ([-pi/2,0,0])*[0,1,0]' + p_object';
p_c_index = r_object * euler2R_XYZ(phi_object)*euler2R_XYZ([pi/3,0,0])*[0,1,0]' + p_object';
p_c_middle = r_object * euler2R_XYZ(phi_object)*euler2R_XYZ([pi*2/3,0,0])*[0,1,0]' + p_object';


[k_thumb,dist_thumb] = dsearchn(Metric.thumb.pos,p_c_thumb');
[k_index,dist_index] = dsearchn(Metric.fingers.pos,p_c_index');
[k_middle,dist_middle] = dsearchn(Metric.middle.pos,p_c_middle');

% index
[M_index,I_index] = max(Metric.fingers.metric_n(k_index,:));
q_index = result_all.fingers{2}{I_index}(k_index,:);
finger_index.update_finger(q_index);
finger_index.print_finger('k',10,12)


[M_thumb,I_thumb] = max(Metric.thumb.metric_n(k_thumb,:));
q_thumb = result_all.thumb{2}{I_thumb}(k_thumb,:);
finger_thumb.update_finger(q_thumb);
finger_thumb.print_finger('k',10,12)

[M_middle,I_middle] = max(Metric.fingers.metric_n(k_middle,:));
q_middle = result_all.fingers{2}{I_middle}(k_middle,:);
finger_middle.update_finger(q_middle');
finger_middle.print_finger('k',10,12)
% plot results

finger_ring.update_finger([0,-0.1,0.4,0.8]);
finger_ring.print_finger('k',10,12)
finger_little.update_finger([0,0,-0.2,0.3,0.5]);
finger_little.print_finger('k',10,12)

plot3(0,0,0,'.')


[X,Y,Z] = sphere;
X = r_object*X + p_object(1);
Y = r_object*Y + p_object(2);
Z = r_object*Z + p_object(3);

surf(X,Y,Z,'FaceColor','b','FaceAlpha',0.5)

plot3(p_c_thumb(1),p_c_thumb(2),p_c_thumb(3),'.','Color','r','MarkerSize',100)
hold on
plot3(p_c_index(1),p_c_index(2),p_c_index(3),'.','Color','r','MarkerSize',100)
hold on
plot3(p_c_middle(1),p_c_middle(2),p_c_middle(3),'.','Color','r','MarkerSize',100)


% xlim([0.02 .2])
% ylim([-0.02 .06])
% zlim([-0.02 .12])
xticks([])
yticks([])
zticks([])
title(fval)
xlabel('x')
ylabel('y')
zlabel('z')
axis equal


%% plot fingers

h = figure(2);
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

finger_index.update_finger([0,0.5,0.4,0.2]);
finger_index.print_finger('k',10,12)

finger_middle.update_finger([0,0.5,0.4,0.2]);
finger_middle.print_finger('k',10,12)

finger_thumb.update_finger([0.1,1,0,0.3,0.3]);
finger_thumb.print_finger('k',10,12)

finger_ring.update_finger([0,-0.1,0.4,0.8]);
finger_ring.print_finger('k',10,12)
finger_little.update_finger([0,0,-0.2,0.3,0.5]);
finger_little.print_finger('k',10,12)

plot3(0,0,0,'.')


[X,Y,Z] = sphere(15);
X = r_object*X + p_object(1);
Y = r_object*Y + p_object(2);
Z = r_object*Z + p_object(3);

surf(X,Y,Z,'FaceColor','b','FaceAlpha',0.5,'EdgeAlpha',0.5)

% plot3(p_c_thumb(1),p_c_thumb(2),p_c_thumb(3),'.','Color','r','MarkerSize',100)
% hold on
% plot3(p_c_index(1),p_c_index(2),p_c_index(3),'.','Color','r','MarkerSize',100)
% hold on
% plot3(p_c_middle(1),p_c_middle(2),p_c_middle(3),'.','Color','r','MarkerSize',100)
% 

% xlim([0.02 .2])
% ylim([-0.02 .06])
% zlim([-0.02 .12])
xticks([])
yticks([])
zticks([])
title(fval)
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

%% plot contact points

h = figure(3);
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

t = linspace(0,2*pi);
x_input = r_object*cos(t);
y_input = r_object*sin(t);

% surf(X,Y,Z,'FaceColor','b')
plot(x_input,y_input,'b-','LineWidth',4)
hold on
plot(r_object*cos(pi/3),r_object*sin(pi/3),'.','Color','r','MarkerSize',150)
hold on
plot(r_object*cos(pi*2/3),r_object*sin(pi*2/3),'.','Color','r','MarkerSize',150)
hold on
plot(r_object*cos(-pi/2),r_object*sin(-pi/2),'.','Color','r','MarkerSize',150)

set(gca,'xticklabel',[],'xtick',[])
set(gca,'yticklabel',[],'ytick',[])
set(gca,'zticklabel',[],'ztick',[])
set(gca,'Visible','off')



axis equal

%% plot

h = figure(13);
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

for i = 1:3
    if i == 1
        metric_i = Metric.fingers;
        finger_analysis = finger_index;
        result = result_all.fingers;
    elseif i==2
        metric_i = Metric.thumb;
        finger_analysis = finger_thumb;
        result = result_all.thumb;
    elseif i==3
        metric_i = Metric.middle;
        finger_analysis = finger_index;
        result = result_all.fingers;
    end
    pos_sample_r = metric_i.pos;
    status = result{9};
    
    
    for subplot_i = 4:4
        
        %     Pos_Car_occupied = zeros(dim_Car+2);
%         subplot(1,4,subplot_i)
        %     figure(22)
        
%         finger_analysis.update_finger(0*finger_analysis.q_a);
        %     finger_analysis.print_finger('g')
%         finger_analysis.update_finger([finger_analysis.limits_q(2:4,2)]);
        %     finger_analysis.print_finger('g')
        
        switch subplot_i
            case 3
                %             metric_term = normalize_vector(joint_limits_index_normalized .* penal_acc_r.* force_pol_index_normalized,1);
                metric_term = metric_i.jl_n;
                title_plot = 'Joint limits';
            case 2
                %             metric_term = normalize_vector(penal_acc_r.* force_pol_index_normalized,1);
                metric_term = metric_i.acc_n;
                title_plot = 'Acc. radius';
            case 1
                metric_term = metric_i.fi_n;
                title_plot = 'Force index';
            case 4
                metric_term = metric_i.metric_n;
                title_plot = 'metric index';
            otherwise
                disp('other value')
        end
        
        
        % plot
        % slice in Cartesian space
        dis_plane = min(pos_sample_r(:,1)):0.01:max(pos_sample_r(:,1));
        pos_condition = false;
        for condi_index = 1:length(dis_plane)
            pos_condition = pos_condition | abs(pos_sample_r(:,1)-dis_plane(condi_index)) < 0.0018;
        end
        
        index_vec_plot = find(status(:) >= 1 & pos_condition); %  & abs(pos_sample_r(:,2)-0.0) < 0.0015
        [~,I_metric_term] = max(metric_term(index_vec_plot,:),[],2);
        
        metric_term_sel = metric_term(index_vec_plot,:);
        
        row = [1:length(index_vec_plot)]';
        col = I_metric_term;
        sz = size(metric_term_sel);
        ind = sub2ind(sz,row,col);
        
        C = [metric_term_sel(ind) zeros(length(index_vec_plot),1) -metric_term_sel(ind)] + [0 0 1];
        
        %     index_vec = find(status(:) >= 1 ); % & abs(pos_sample_r(:,1)-0.056) < 0.0015
        %     C = [metric_term(index_vec) zeros(length(index_vec),1) -metric_term(index_vec)] + [0 0 1];
        
        
        % origin finger
        hs = scatter3(pos_sample_r(index_vec_plot,1),pos_sample_r(index_vec_plot,2),pos_sample_r(index_vec_plot,3),30,...
            C,'square','filled','MarkerEdgeAlpha',.6,'MarkerFaceAlpha',.6);
        
        hold on
        
        
        
%         index_vec = find(status(:) >= 1);
%         metric_term_sel = metric_term(index_vec,:);
%         [~,I] = max(metric_term_sel(:));
%         [I_x,I_y]=ind2sub(size(metric_term_sel),I);
%         q_i = result{2}{I_y}(index_vec(I_x),:)';
%         finger_analysis.update_finger(q_i);
%         finger_analysis.print_finger('k',5,8);
        
%         fprintf("       JL,     Acc.R.,     F.I.,     M: \n")
%         fprintf("Index: %f, %f, %f, %f \n", metric_i.jl_n(index_vec(I_x),I_y), metric_i.acc_n(index_vec(I_x),I_y),...
%              metric_i.fi_n(index_vec(I_x),I_y), metric_i.metric_n(index_vec(I_x),I_y))
%         %
        axis equal
        grid off
        xlabel('x')
        ylabel('y')
        zlabel('z')
        title(title_plot)
        
    end
end

xlim([0 .22])
ylim([-0.04 .14])
zlim([-0.08 .14])
xticks([0.02 0.08 0.14 0.2])
yticks([-0 0.06 0.12])
zticks([-0.06 0 0.04 0.1])









