% Data analysis and map visualization 




% Junnan Li, junnan.li@tum.de, MIRMI, 06.2023

%% load work space
clear all
% close all 
clc

% load ./optimization/results/workspace_2806_x_4mm_30_shadow.mat % shadow hand data 
% load ./optimization/results/workspace_2606_x_3mm_30_shadow_index.mat
% load ./optimization/results/variable_2606_x_3mm_30_shadow_thumb.mat result

% load ./optimization/results/workspace_3006_x_3mm_30_shadow.mat % shadow hand data with changed d_0 
load ./optimization/results/workspace_3006_x_3mm_30_shadow_fixed.mat
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
    
    max_acc = zeros(1,2);
    max_fi = zeros(1,2);
    
    for i_ns = 1:2
        
        % acceleration index
        mani_acc_r = result{5}{i_ns};
        %     acc_r_max = max(mani_acc_r);
        %     acc_r_min = min(mani_acc_r);
        %     acc_r_index_normalized = (mani_acc_r - acc_r_min)/(acc_r_max-acc_r_min);
        
        acc_r_index_normalized(:,i_ns) = normalize_vector(mani_acc_r, 1);
        penal_acc_r(:,i_ns) = penalty_acc_r(acc_r_index_normalized(:,i_ns), 1);
        max_acc(i_ns) = max(mani_acc_r);
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
        
        max_fi(i_ns) = max(index_max_force_pol);
        
        
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
        Metric.fingers.max_fi = max_fi;
        Metric.fingers.max_acc = max_acc;
    elseif i == 2
        Metric.thumb.metric_n = metric_normalized;
        Metric.thumb.fi_n = force_pol_index_normalized;
        Metric.thumb.jl_n = joint_limits_index_normalized;
        Metric.thumb.acc_n = penal_acc_r;
        Metric.thumb.pos = pos_sample_r;
        Metric.thumb.max_fi = max_fi;
        Metric.thumb.max_acc = max_acc;
    elseif i == 3
        Metric.little.metric_n = metric_normalized;
        Metric.little.fi_n = force_pol_index_normalized;
        Metric.little.jl_n = joint_limits_index_normalized;
        Metric.little.acc_n = penal_acc_r;
        Metric.little.pos = pos_sample_r;
        Metric.little.max_fi = max_fi;
        Metric.little.max_acc = max_acc;
    end
    
end

%% results Cartesian space

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





for i = 1
    if i == 1
        metric_i = Metric.fingers;
        finger_analysis = finger_index;
        result = result_all.fingers;
    elseif i==2
        metric_i = Metric.thumb;
        finger_analysis = finger_thumb;
        result = result_all.thumb;
    elseif i==3
        metric_i = Metric.little;
        finger_analysis = finger_little;
        result = result_all.little;
    end
    pos_sample_r = result{1};
    status = result{9};
    
    
    for subplot_i = 1
        
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
%                             metric_term = normalize_vector(metric_i.acc_n.* metric_i.fi_n,1);
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
        
        index_vec_plot = find(status(:) >= 1 & pos_condition); % 
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
        hs = scatter3(pos_sample_r(index_vec_plot,1),pos_sample_r(index_vec_plot,2),pos_sample_r(index_vec_plot,3),70,...
            C,'square','filled','MarkerEdgeAlpha',.5,'MarkerFaceAlpha',.5);
        
        hold on
        
        
        
        index_vec = find(status(:) >= 1);
        metric_term_sel = metric_term(index_vec,:);
        [~,I] = max(metric_term_sel(:));
        [I_x,I_y]=ind2sub(size(metric_term_sel),I);% I_y is the index of nullspace config
        q_i = result{2}{I_y}(index_vec(I_x),:)';

%         finger_analysis.update_finger(q_i);
%         finger_analysis.print_finger('k',10,15);
        
        
        %
        p_c = finger_analysis.get_p_all_links;
%         plot3(p_c(1,end),p_c(2,end),p_c(3,end),'o','Color','g','MarkerSize',20,'LineWidth',15)
        axis equal
        grid on
        
%         finger_thumb.update_finger([0,pi/3,0,pi/10,pi/10]);
%         finger_thumb.print_finger([0.7,0.7,0.7],5,8);
        finger_index.update_finger([0,pi/6,pi/6,pi/10]);
        finger_index.print_finger([0,0,0],5,8);
%         finger_middle.update_finger([0,pi/10,pi/10,pi/15]);
%         finger_middle.print_finger([0.7,0.7,0.7],5,8);
%         finger_ring.update_finger([0,pi/15,pi/15,pi/15]);
%         finger_ring.print_finger([0.7,0.7,0.7],5,8);
%         finger_little.update_finger([0,0,pi/20,pi/20,0.1]');
%         finger_little.print_finger([0,0,0],10,15);


        % get fingertip  metric
        

        % % xlim([0 .2])
% % ylim([-0.02 .06])
% % zlim([-0.02 .12])
        xticks([0.06 0.12 0.18])
        yticks([-0.05 0 0.05 0.1])
        zticks([-0.05 0 0.05 0.1])
        xlabel('x')
        ylabel('y')
        zlabel('z')
%         colorbar
        title(title_plot)
    end
    
    p_j_all = finger_index.get_p_all_links;
    p_fingertip = p_j_all(:,end);
    ind = dsearchn(pos_sample_r,p_fingertip');
    metric_term(ind,I_y)
    fprintf("       JL,     Acc.R.,     F.I.,     M: \n")
    fprintf("Index: %f, %f, %f, %f \n", metric_i.jl_n(ind,I_y), metric_i.acc_n(ind,I_y),...
             metric_i.fi_n(ind,I_y)*metric_i.max_fi(I_y), metric_i.metric_n(ind,I_y))
end




return

%% plot result for two finger

dis_plane = -0.03:0.008:0.1;
pos_condition = false;
for condi_index = 1:length(dis_plane)
    pos_condition = pos_condition | abs(pos_sample_r(:,1)-dis_plane(condi_index)) < 0.0015;
end

index_vec = find(status(:) >= 1 & pos_condition); %  & abs(pos_sample_r(:,2)-0.0) < 0.0015

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



for subplot_i = 1:3

    subplot(1,3,subplot_i)

    finger_analysis.update_finger([0,0,0,0]')
    finger_analysis.update_finger([0;finger_analysis.limits_q(2:4,2)])
    switch subplot_i
        case 3
            metric_term = normalize_vector(joint_limits_index_normalized .* penal_acc_r.* force_pol_index_normalized,1);
            title_plot = 'Force index + Acc. radius + Joint limits';
        case 2
            metric_term = normalize_vector(penal_acc_r.* force_pol_index_normalized,1);
            title_plot = 'Force index + Acc. radius';
        case 1
            metric_term = force_pol_index_normalized;
            title_plot = 'Force index';
        case 4
            metric_term = metric_normalized;
            title_plot = 'metric index';
        otherwise
            disp('other value')
    end
    
    [~,I_metric_term] = max(metric_term(index_vec,:),[],2);

    metric_term_sel = metric_term(index_vec,:);

    row = [1:length(index_vec)]';
    col = I_metric_term;
    sz = size(metric_term_sel);
    ind = sub2ind(sz,row,col);

    C = [metric_term_sel(ind) zeros(length(index_vec),1) -metric_term_sel(ind)] + [0 0 1];
   
    % origin finger
    hs = scatter3(pos_sample_r(index_vec,1),pos_sample_r(index_vec,2),pos_sample_r(index_vec,3),60,...
        C,'square','filled','MarkerEdgeAlpha',.8,'MarkerFaceAlpha',.8);
    
    hold on
    [~,I] = max(metric_term_sel(:));
    [I_x,I_y]=ind2sub(size(metric_term_sel),I);
    q_i = result{2}{I_y}(index_vec(I_x),:)';
    finger_analysis.update_finger(q_i);
    finger_analysis.print_finger('k',8,12);

    fprintf("       JL,     Acc.R.,     F.I.,     M: \n")
    fprintf("Index: %f, %f, %f, %f \n", joint_limits_index_normalized(index_vec(I_x),I_y), penal_acc_r(index_vec(I_x),I_y),...
                    force_pol_index_normalized(index_vec(I_x),I_y), metric_normalized(index_vec(I_x),I_y))   
%     
    axis equal
    grid on

    

    xlabel('x')
    ylabel('y')
    zlabel('z')
    title(title_plot)

end

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

i = 68961;
% index = 6207;

% index = find(abs(pos_sample_r(:,1)-0.077) < 0.0015 & abs(pos_sample_r(:,2)-0.0) < 0.0015& ...
%     abs(pos_sample_r(:,3)- (-0.016)) < 0.0015 )
q_sample_1 = result{2}{1}(i,:);
q_sample_2 = result{2}{2}(i,:);
force_index_1 = result{3}{1}(i,:);
force_index_2 = result{3}{2}(i,:);
f_vol_1 = result{4}{1}(i,:);
f_vol_2 = result{4}{2}(i,:);
acc_r_1 = result{5}{1}(i,:);
acc_r_2 = result{5}{2}(i,:);
pos_sample_r = result{1}(i,:);
status = result{9}(i,:);

finger_analysis.update_finger(q_sample_1);
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
[~,M_r,~,~] = finger_analysis.fordyn_ne_w_end(q_i, zeros(finger_analysis.nja,1), zeros(finger_analysis.nja,1), zeros(6,finger_analysis.nja+2));
P_acc = Polyhedron('V',(J_index_red * inv(M_r) * P_tau.V')');
P_acc.minHRep;
P_acc.minVRep;
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
P_ee.minHRep;
subplot(1,3,3)
P_ee.plot
axis equal
% calculate the force polytope
if direction_downward
    half_plane_vector = [0,0,1];
else
    half_plane_vector = [0,0,-1];
end
for mi = 1:9 % direction index
    % reduce the dimension from 3 to 1 along the direction
    % vector
    half_plane_2 = -cross(Ae_all(2*mi-1,:),Ae_all(2*mi,:));
    p_tmp_force = Polyhedron('A', [P_ee.A;half_plane_vector;half_plane_2], 'b',[P_ee.b;0;0], 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
    p_tmp_force.minHRep;
    Vertex_f_i = p_tmp_force.V;
    if isempty(Vertex_f_i)
        max_pos_f_Ver_i = 0;
    else
        max_pos_f_Ver_i = max(diag(sqrt(Vertex_f_i*Vertex_f_i'))) % vextex of the 1-dimension polytope
    end
end

