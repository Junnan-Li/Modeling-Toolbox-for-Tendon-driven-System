% generate the manipulability maps of the finger objects


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
%%%%%
% 
% return

%% get finger information

nj = finger_index.nja;
% A = [eye(4);-1*eye(4)];
% b = ones(2*nj,1);
% P_tau = Polyhedron('A', A, 'b', b);

% get coupling matrix
M_coupling = finger_index.M_coupling;
% get force limits
force_limits = finger_index.limits_ft;

% generate the torque polytope
[P_tau, P] = polytope_torque(M_coupling, force_limits);

% customized jonit limits
q_limit = [-10,10;0,90;0,80;0,40];

% voxelize the joint space
[q1,q2,q3,q4] = ndgrid(q_limit(1,1):5:q_limit(1,2),...
    q_limit(2,1):10:q_limit(2,2),...
    q_limit(3,1):10:q_limit(3,2),...
    q_limit(4,1):10:q_limit(4,2));
% number of the voxels
n_sample = size(q1,1)*size(q1,2)*size(q1,3)*size(q1,4);

% create variables for the results 
mani_index_r = zeros(n_sample,5);
mani_index_r_acc = zeros(n_sample,5);
pos_sample_r = zeros(n_sample,3);
q_sample_r = zeros(n_sample,4);
vol_sample_r = zeros(n_sample,1);
vol_sample_r_acc = zeros(n_sample,1);
r_sample_r_acc = zeros(n_sample,1);

% struct for saving the results 
opt_act_method = struct();
opt_act_method.Metric_force = zeros(n_sample,5);
opt_act_method.Metric_acc = zeros(n_sample,5);
opt_act_method.end_pos = zeros(n_sample,3);
opt_act_method.Q = zeros(n_sample,4);
opt_act_method.vol_force = zeros(n_sample,1);
opt_act_method.vol_acc = zeros(n_sample,1);
opt_act_method.r_acc = zeros(n_sample,1);
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
                
                % check the Singularity
                if rank(J_index_red) < 3
                    mani_index_r(index,:) = -0*ones(1,5);
                    index = index + 1;
                    continue
                end

                % calculate the Mass matrix 
                [~,M_r,~,~] = finger_index.fordyn_ne_w_end(q_i, zeros(4,1), zeros(4,1), zeros(6,6));
                % calculate the Cartesian acceleration polytope
                P_acc = J_index_red * inv(M_r) * P_tau;
                
                % calculate the volume of the acceleration polytope
                try
                    vol_sample_r_acc(index) = P_acc.volume;
                    r_sample_r_acc(index) = largest_minimum_radius_P_input(P_acc, [0,0,0]');
                    if P_acc.volume < 0.005
                        fprintf('asd\n')
                    end

                catch
                    vol_sample_r_acc(index) = 0;
                    r_sample_r_acc(index) = 0;
                end
                % calculate the force polytope
                P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
                
                try
                    vol_sample_r(index) = P_ee.volume;
                catch
                    %                     warning('Problem using function.  Assigning a value of 0.');
                    vol_sample_r(index) = 0;
                end
                
                % calculate the force polytope
                for mi = 1:5 % direction index
                    % reduce the dimension from 3 to 1 along the direction
                    % vector
                    p_tmp_force = Polyhedron('A', P_ee.A, 'b', P_ee.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
                    vec_tmp_force = p_tmp_force.distance(1000*direction_vec(mi,:)').y;
                    dist_tmp_force = p_tmp_force.distance(1000*direction_vec(mi,:)').dist;
                    
                    p_tmp_acc = Polyhedron('A', P_acc.A, 'b', P_acc.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
                    vec_tmp_acc = p_tmp_acc.distance(1000*direction_vec(mi,:)').y;
                    dist_tmp_acc = p_tmp_acc.distance(1000*direction_vec(mi,:)').dist;
                    if dist_tmp_force >= 1000
                        mani_index_r(index,mi) = 0;
                    else
                        mani_index_r(index,mi) = sqrt(vec_tmp_force'*vec_tmp_force);
                    end
                    if dist_tmp_acc >= 1000
                        mani_index_r_acc(index,mi) = 0;
                    else
                        mani_index_r_acc(index,mi) = sqrt(vec_tmp_acc'*vec_tmp_acc);
                    end

                end
                % test(10,J_index_red,P_ee,P_tau,p_tmp,finger_index); % test use
                fprintf('processing: %d of %d \n',index, n_sample)
                index = index + 1;

            end
        end
    end
end

opt_act_method.Metric_force = mani_index_r;
opt_act_method.Metric_acc = mani_index_r_acc;
opt_act_method.end_pos = pos_sample_r;
opt_act_method.Q = q_sample_r;
opt_act_method.vol_force = vol_sample_r;
opt_act_method.vol_acc = vol_sample_r_acc;
opt_act_method.r_acc = r_sample_r_acc;


%%% save data
% save('./test/optimization/workspace_1705.mat');


return
%% debug
index_i = 2224;

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
P_ee_acc = 100*J_index_red* inv(M_r)*P_tau;
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

%% plot results
% index_max = max(mani_index(:));
% weight = [2 1 1 1 1];
% weighted_index_r = mani_index_r * weight';

mani_force_polytope = mani_index_r;
mani_acc_polytope = mani_index_r_acc;

% acceleration index
mani_acc_r = r_sample_r_acc;
acc_r_max = max(r_sample_r_acc);
acc_r_min = min(r_sample_r_acc);
acc_r_index_normalized = (mani_acc_r - acc_r_min)/(acc_r_max-acc_r_min);

% joint angle index
q_limit_low = q_limit(:,1);
q_limit_upper = q_limit(:,2);
mani_q = q_sample_r*180/pi;
mani_q_normalized = (mani_q - q_limit_low')./(q_limit_upper'-q_limit_low');

% use sin wave for joint limits 
joint_limits_index_sin = sin(mani_q_normalized*pi)+1;
joint_limits_index = (joint_limits_index_sin(:,1).*joint_limits_index_sin(:,2).*...
                    joint_limits_index_sin(:,3).*joint_limits_index_sin(:,4));

joint_limits_index_normalized = (joint_limits_index-min(joint_limits_index))/...
                                (max(joint_limits_index)-min(joint_limits_index));


figure(10)
for j = 1:5
    index_max_force_pol = max(mani_force_polytope(:,j));
    index_min_force_pol = min(mani_force_polytope(:,j));

    index_max_acc_pol = max(mani_acc_polytope(:,j));
    index_min_acc_pol = min(mani_acc_polytope(:,j));
    %
    mani_index_plot_j = mani_force_polytope(:,j);
    mani_acc_pol_j = mani_acc_polytope(:,j);

    force_pol_index_normalized = (mani_index_plot_j - index_min_force_pol)/(index_max_force_pol-index_min_force_pol);
    acc_pol_index_normalized = (mani_acc_pol_j - index_min_acc_pol)/(index_max_acc_pol-index_min_acc_pol);

    metric_j = joint_limits_index_normalized .* acc_r_index_normalized.* ...
               acc_pol_index_normalized.* force_pol_index_normalized;
%     metric_j = joint_limits_index_normalized;

    metric_j_max = max(metric_j);
    metric_j_min = min(metric_j);
    metric_j_normalized = (metric_j-metric_j_min)/(metric_j_max-metric_j_min);


    subplot(2,3,j)
    for i = 1:n_sample
        p_link_all_w_r = pos_sample_r(i,:);
        plot3(p_link_all_w_r(1),p_link_all_w_r(2),p_link_all_w_r(3),'.',...
            'Color',[1 0 0]*metric_j_normalized(i));
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







