% generate the manipulability maps of the finger objects
% second version


% Junnan Li, junnan.li@tum.de, MIRMI, 05.2023



clear all
close all
clc


%% 
% create the index finger and thumb 
run opt_create_thumb_index.m

close all

%%%%% test poses
% q = [-15;0;0;0]*pi/180;
% q_t = [0;0;0;0]*pi/180;
% finger_index.update_finger(q);
% finger_thumb.update_finger(q_t);
% finger_index.update_list_contacts;
% figure(2)
% finger_index.print_finger
% % finger_thumb.print_finger
% q = [15;0;0;0]*pi/180;
% finger_index.update_finger(q);
% finger_index.print_finger
% grid on
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
%%%%
% 
% return

%% get finger information

nj = finger_index.nja;
% get coupling matrix
M_coupling = finger_index.M_coupling;
% get force limits
force_limits = finger_index.limits_ft;

% generate the torque polytope
[P_tau, P] = polytope_torque(M_coupling, force_limits);

% customized jonit limits
q_limit = [-15,15;0,80;0,80;0,50];
for i = 1:nj
    finger_index.list_joints(i).q_limits = q_limit(i,:)*pi/180;
end
finger_index.update_joints_info;


% voxelize the joint space
[q1,q2,q3,q4] = ndgrid(q_limit(1,1):3:q_limit(1,2),...
    q_limit(2,1):5:q_limit(2,2),...
    q_limit(3,1):5:q_limit(3,2),...
    q_limit(4,1):5:q_limit(4,2));
% number of the voxels
n_sample = size(q1,1)*size(q1,2)*size(q1,3)*size(q1,4);

% create variables for the results 
force_index = zeros(n_sample,5);
acc_index = zeros(n_sample,5);
pos_sample_r = zeros(n_sample,3);
q_sample_r = zeros(n_sample,4);
vol_sample_force = zeros(n_sample,1);
vol_sample_acc = zeros(n_sample,1);
r_sample_acc = zeros(n_sample,1);
cond_JM = zeros(n_sample,1);
cond_J = zeros(n_sample,1);

states_sample = zeros(n_sample,1);
% states: 1: normal, 2: 
%       11: Jacobian deficient
%       21: acceleration polytope volume too large
%       29: unable to get acceleration volume
%       39: unable to get force volume

% struct for saving the results 
opt_act_method = struct();
opt_act_method.Metric_force = zeros(n_sample,5); % force index
opt_act_method.Metric_acc = zeros(n_sample,5); % unused
opt_act_method.end_pos = zeros(n_sample,3);
opt_act_method.Q = zeros(n_sample,4);
opt_act_method.vol_force = zeros(n_sample,1);
opt_act_method.vol_acc = zeros(n_sample,1);
opt_act_method.r_acc = zeros(n_sample,1);
opt_act_method.cond_JM = zeros(n_sample,1);
opt_act_method.cond_J = zeros(n_sample,1);
opt_act_method.states_sample = zeros(n_sample,1);
% 

index = 1;
%% force direction definition of index finger
% directions: 5
% Ae_all [2*5x3], the vectors that are orthogonal to the direction vectors
SQ2 = sqrt(2)/2;
Ae_all = [1 0 0;...
          0 1 0;...
          SQ2 0 SQ2;...
          0 1 0;...
          SQ2 0 -SQ2;...
          0 1 0;...
          1 0 0;...
          0 SQ2 SQ2;...
          1 0 0;...
          0 SQ2 -SQ2];
be_all = zeros(10,1);

% 5 direction vectors
direction_vec = [0 0 -1;...
    SQ2 0 -SQ2;...
    -SQ2 0 -SQ2;...
    0 SQ2 -SQ2;...
    0 -SQ2 -SQ2];
%%

figure(3)
grid on
axis equal
for i = 1:size(q1,1)
    for j = 1: size(q1,2)
        for k = 1:size(q1,3)
            for h = 1:size(q1,4)
                % update the current q
                q_i = [q1(i,j,k,h);q2(i,j,k,h);q3(i,j,k,h);q4(i,j,k,h)]*pi/180;
                finger_index.update_finger(q_i); 
                % print the finger in the plot
                finger_index.print_finger;
                hold off
                drawnow
                
                % get the joint position
                pos_i = finger_index.get_p_all_links;
                % update the variables
                pos_sample_r(index,:) = pos_i(:,end)';
                q_sample_r(index,:) = q_i';

                % calculate the index
                % get Jacobian
                J_index = finger_index.Jacobian_analytic_b_end;
                % get reduced Jacobian only with translational terms
                J_index_red = finger_index.w_R_base * J_index(1:3,:);
                
                % check the Singularity of Jacobian
                if rank(J_index_red) < 3
                    force_index(index,:) = -1*ones(1,5);
                    acc_index(index) = -1;
                    vol_sample_force(index) = -1;
                    vol_sample_acc(index) = -1;
                    r_sample_acc(index) = -1;
                    index = index + 1;
                    states_sample(index) = 11;
                    continue
                end

                % calculate the Mass matrix 
                [~,M_r,~,~] = finger_index.fordyn_ne_w_end(q_i, zeros(4,1), zeros(4,1), zeros(6,6));
                % calculate the Cartesian acceleration polytope
                P_acc = J_index_red * inv(M_r) * P_tau;
                
                % check condition number of J*M-1
                cond_JM(index) = cond(J_index_red * inv(M_r));
                cond_J(index) = cond(J_index_red);
                % calculate the volume of the acceleration polytope
                try
                    vol_sample_acc(index) = P_acc.volume;
                    r_sample_acc(index) = largest_minimum_radius_P_input(P_acc, [0,0,0]');
                    if P_acc.volume < 0.005
                        fprintf('asd\n')
                    elseif P_acc.volume > 100
                        states_sample(index) = 21;
                    end

                catch
                    vol_sample_acc(index) = 0;
                    r_sample_acc(index) = 0;
                    states_sample(index) = 29;
                end
                
                % calculate the force polytope
                P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
                
                try
                    vol_sample_force(index) = P_ee.volume;
                catch
                    % warning('Problem using function.  Assigning a value of 0.');
                    vol_sample_force(index) = 0;
                    states_sample(index) = 39;
                end
                
                % calculate the force polytope
                for mi = 1:5 % direction index
                    % reduce the dimension from 3 to 1 along the direction
                    % vector
                    p_tmp_force = Polyhedron('A', P_ee.A, 'b', P_ee.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
                    Vertex_f_i = p_tmp_force.V;
                    max_pos_f_Ver_i = max(Vertex_f_i(:,3)); % vextex of the 1-dimension polytope
                    if max_pos_f_Ver_i < 0
                        force_index(index,mi) = -1;
                    else
                        force_index(index,mi) = max_pos_f_Ver_i;
                    end
                    % acceleration along the axis
                    p_tmp_acc = Polyhedron('A', P_acc.A, 'b', P_acc.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
                    Vertex_acc_i = p_tmp_acc.V;
                    max_pos_acc_Ver_i = max(Vertex_acc_i(:,3)); % vextex of the 1-dimension polytope
                    if max_pos_acc_Ver_i < 0
                        acc_index(index,mi) = -1;
                    else
                        acc_index(index,mi) = max_pos_acc_Ver_i;
                    end
                   
                end
                % test(10,J_index_red,P_ee,P_tau,p_tmp,finger_index); % test use
                fprintf('processing: %d of %d \n',index, n_sample)
                index = index + 1;

            end
        end
    end
end

opt_act_method.Metric_force = force_index;
opt_act_method.Metric_acc = acc_index;
opt_act_method.end_pos = pos_sample_r;
opt_act_method.Q = q_sample_r;
opt_act_method.vol_force = vol_sample_force;
opt_act_method.vol_acc = vol_sample_acc;
opt_act_method.r_acc = r_sample_acc;
opt_act_method.cond_JM = cond_JM;
opt_act_method.cond_J = cond_J;
opt_act_method.states_sample = states_sample;

%%% save data
% save('./test/optimization/workspace_2305_5deg.mat');


return
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
metric_value = zeros(size(mani_force_polytope));
metric_normalized = zeros(size(mani_force_polytope));


for j = 1:5
    % normalize the force index of one direction
    index_max_force_pol = max(mani_force_polytope(:,j));
    index_min_force_pol = min(mani_force_polytope(:,j));
    %
    mani_index_plot_j = mani_force_polytope(:,j);
    
    force_pol_index_normalized = (mani_index_plot_j - index_min_force_pol)/(index_max_force_pol-index_min_force_pol);
    
    metric_j = joint_limits_index_normalized .* acc_r_index_normalized.* ...
               acc_v_index_normalized.* force_pol_index_normalized;

    metric_value(:,j) = metric_j;  
    % normalized
    metric_j_max = max(metric_j);
    metric_j_min = min(metric_j);
    metric_j_normalized = (metric_j-metric_j_min)/(metric_j_max-metric_j_min);
    
    metric_normalized(:,j) = metric_j_normalized;
end


%% plot Joint space


figure(10)
for j = 1
    subplot(2,3,j)
    for i = 1:n_sample
        p_link_all_w_r = pos_sample_r(i,:);
        plot3(p_link_all_w_r(1),p_link_all_w_r(2),p_link_all_w_r(3),'.',...
            'Color',[1 0 0]*metric_normalized(i),'MarkerSize',2);
        hold on
        %         p_link_all_w_t = pos_sample_t(i,:);
        %         plot3(p_link_all_w_t(1),p_link_all_w_t(2),p_link_all_w_t(3),'.',...
        %             'Color',[0 1 0]*(mani_index_t(i,j)-index_min_t)/(index_max_t-index_min_t));
        %         hold on
    end
    grid on
    title(strcat('direction',num2str(j)))
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
end
subplot(2,3,6)
for i = 1:5
    plot3([0;direction_vec(i,1)],[0;direction_vec(i,2)],[0;direction_vec(i,3)],'-');
    hold on
end
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
grid on
legend('1','2','3','4','5')
title('all directions')


%% results Cartesian space
C_min = min(pos_sample_r);
C_max = max(pos_sample_r);
voxel_length = 0.003;
dim_Car = floor((C_max-C_min)/voxel_length); % 0.01m 

voxel_min = floor(C_min/voxel_length);

Pos_Car_occupied = zeros(dim_Car+2);


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
    
    Pos_Car_occupied = zeros(dim_Car+2);
    subplot(2,3,subplot_i)
    
    finger_index.update_finger([0,0,0,0]')
    finger_index.print_finger('g')
    
    switch subplot_i
        case 1
            metric_term = joint_limits_index_normalized;
            title_plot = 'Joint limits';
        case 2
            metric_term = acc_r_index_normalized;
            title_plot = 'Acc. radius';
        case 3
            metric_term = acc_pol_index_normalized;
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
    
    
    for i = 1:size(pos_sample_r,1)
        pos_i = pos_sample_r(i,:);
        index_i = floor(pos_i/voxel_length)- voxel_min + 1;
        
        %         metric_i_m = metric_j_normalized(i);
        %         metric_i_jl = joint_limits_index_normalized(i);
        %         metric_i_accr = acc_r_index_normalized(i);
        %         metric_i_accp = acc_pol_index_normalized(i);
        %         metric_i_fp = force_pol_index_normalized(i);
        metric_i = metric_term(i);
        
        metric_i_ori = Pos_Car_occupied(index_i(1),index_i(2),index_i(3));
        if metric_i > metric_i_ori
            Pos_Car_occupied(index_i(1),index_i(2),index_i(3)) = metric_i;
        end
    end
    
    for i_x = 1:size(Pos_Car_occupied,1)
        for i_y = 1:size(Pos_Car_occupied,2)
            for i_z = 1:size(Pos_Car_occupied,3)
                voxel_i_occupied = Pos_Car_occupied(i_x,i_y,i_z);
                if ~voxel_i_occupied
                    continue
                end
                voxel_i_pos = (voxel_min+[i_x,i_y,i_z]-1)*voxel_length + voxel_length/2;
                if abs(voxel_i_pos(2)-1e-3) < 2e-3
                    plot3(voxel_i_pos(1),voxel_i_pos(2),voxel_i_pos(3),'.','Color',...
                        [1 0 -1]*voxel_i_occupied + [0 0 1],'MarkerSize',10);
                    hold on
                end
                
            end
        end
    end
    [~,I] = max(metric_term);
    q_i = q_sample_r(I,:)';
    finger_index.update_finger(q_i);
    finger_index.print_finger();
    
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    title(title_plot)
    
end

%% debug
index_i = 16358;

q = opt_act_method.Q(index_i,:)';
finger_index.update_finger(q);
figure(100)
finger_index.print_finger;
grid on
axis equal
% manipulability index
J_index = finger_index.Jacobian_analytic_b_end;
J_index_red = finger_index.w_R_base * J_index(1:3,:);
fprintf('cond %f.2 \n', cond(J_index_red))
% svd(J_index_red)
P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
figure(101)
P_ee.plot('alpha',0.5);
hold on

[~,M_r,~,~] = finger_index.fordyn_ne_w_end(q, zeros(4,1), zeros(4,1), zeros(6,6));
P_ee_acc = J_index_red* inv(M_r)*P_tau;
% figure(102)
P_ee_acc.plot('alpha',0.3,'Color','b');
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
for mi = 1:5
    p_tmp_force = Polyhedron('A', P_ee.A, 'b', P_ee.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
    vec_tmp_force = p_tmp_force.distance(1000*direction_vec(mi,:)').y;
    dist_tmp_force = p_tmp_force.distance(1000*direction_vec(mi,:)').dist;

    p_tmp_acc = Polyhedron('A', P_acc.A, 'b', P_acc.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
    vec_tmp_acc = p_tmp_acc.distance(1000*direction_vec(mi,:)').y;
    dist_tmp_acc = p_tmp_acc.distance(1000*direction_vec(mi,:)').dist;
    %     if dist_tmp_force >= 1000
%         mani_index_r(index,mi) = 0;
%     else
%         mani_index_r(index,mi) = sqrt(vec_tmp_force'*vec_tmp_force);
%     end
%     if dist_tmp_acc >= 1000
%         mani_index_r_acc(index,mi) = 0;
%     else
%         mani_index_r_acc(index,mi) = sqrt(vec_tmp_acc'*vec_tmp_acc);
%     end
end

% 
% figure(101)
% hold on
% quiver3(zeros(5,1),zeros(5,1),zeros(5,1),30*direction_vec(:,1),30*direction_vec(:,2),30*direction_vec(:,3),'LineWidth',3)

%% debug inverse kinematic

q_ori = q_sample_r(11700,:)';
x = pos_sample_r(11700,:)';
[q_res, ~, ~,phi_x,~] = finger_index.invkin_trans_numeric(x,1000,1e-5,0.1)
finger_index.update_finger([0,.5,0.2,0.5]')
q_res2 = finger_index.invkin_trans_numeric_joint_limits(x,1000,1e-5,0.1)

%% local functions
% 

function status = test(i,J_index_red,P_ee,P_tau,p_tmp,finger_index)
% close all
rank(J_index_red)
figure(i+1)
P_ee.plot;
hold on
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
figure(i+2)
p_tmp.plot;
hold on
xlabel('x')
ylabel('y')
zlabel('z')
figure(i+3)
finger_index.print_finger;
grid on
axis equal
hold on
xlabel('x')
ylabel('y')
zlabel('z')
P_tau3 = [zeros(3,1),eye(3)] * P_tau;

figure(i+4)
P_tau3.plot;
hold on
xlabel('x')
ylabel('y')
zlabel('z')
status = 1;
end







