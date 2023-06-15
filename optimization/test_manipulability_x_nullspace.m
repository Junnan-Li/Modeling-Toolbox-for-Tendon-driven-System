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


%% inverse kinematic test
% customized jonit limits

% q_limit = [-15,15;0,80;0,80;0,70];
% for i = 1:finger_index.nj
%     finger_index.list_joints(i).q_limits = q_limit(i,:)*pi/180;
% end
% finger_index.update_joints_info;
%
% q = [0;0;0;0]*pi/180;
% finger_index.update_finger(q);
% q1 = [10;70;0;70]*pi/180;
% finger_index.update_finger(q1);
%
% p_link_all_w_r = finger_index.get_p_all_links;
% x_end_d = p_link_all_w_r(:,end);
% %%%%% test poses
%
% figure(1)
% plot3(finger_index.w_p_base(1),finger_index.w_p_base(2),finger_index.w_p_base(3),'x','MarkerSize',15);
% hold on
% plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','r');
% hold on
%
% finger_index.update_finger(q);
% iter_max = 100;
% alpha = 0.8;
% color_plot = [1,0,0];
% tol = 1e-9;
% [q,q_all,x_res,phi_x,iter] = finger_index.invkin_trans_numeric_joint_limits(x_end_d,iter_max,tol,alpha);
%
%
% finger_index.update_finger(q);
% p_link_all_w_r = finger_index.get_p_all_links;
% plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color','b');
% hold on
%
% grid on
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')

% return

%% get finger information
% finger to analyze
%       finger_index: 8 tendons
%       finger_index_5_tendon: 5 tendons
%       finger_thumb


finger_analysis = finger_index;

% downward (1) or upward 0
direction_downward = 1; 

%%

nj = finger_analysis.nja;
% get coupling matrix
M_coupling = finger_analysis.M_coupling;
% get force limits
force_limits = finger_analysis.limits_ft;

% generate the torque polytope
[P_tau, P] = polytope_torque(M_coupling, force_limits);

% customized jonit limits
q_limit = [-15,15;0,90;0,90;0,50];
for i = 1:nj
    finger_analysis.list_joints(i).q_limits = q_limit(i,:)*pi/180;
end
finger_analysis.update_joints_info;

% get cartesian workspace cube
% voxelize the joint space
finger_len = sum(finger_analysis.mdh_ori.a);
p_base = finger_analysis.w_p_base;
% workspace_cube_ori = [-0.05;-0.02;-0.09]; % thumb
workspace_cube_ori = [-0.025;-0.026;-0.11]; % index

% size of the voxel
voxel_length = 0.004;


[x_voxel,y_voxel,z_voxel] = ndgrid(workspace_cube_ori(1):voxel_length:workspace_cube_ori(1)+0.13,...
    workspace_cube_ori(2):voxel_length:workspace_cube_ori(2)+0.054,...
    workspace_cube_ori(3):voxel_length:workspace_cube_ori(3)+0.17);

n_sample = size(x_voxel,1)*size(y_voxel,2)*size(z_voxel,3);


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




%% init variables
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

result = {zeros(n_sample,3),{zeros(n_sample,4),zeros(n_sample,4)},{zeros(n_sample,5),zeros(n_sample,5)},...
            {zeros(n_sample,1),zeros(n_sample,1)},{zeros(n_sample,1),zeros(n_sample,1)},{zeros(n_sample,1),zeros(n_sample,1)},...
            {zeros(n_sample,1),zeros(n_sample,1)},{zeros(n_sample,1),zeros(n_sample,1)},zeros(n_sample,1)};

index = 1;

% parameters of inverse kinematic
iter_max = 1000;
alpha = 0.7;
tol = 1e-8;
q_diff_min = 5*pi/180;

%% generAte the pos_sample_r vector 

x_all_samples = zeros(n_sample,3);
for i = 1:size(x_voxel,1)
    for j = 1: size(y_voxel,2)
        for k = 1:size(z_voxel,3)
            fprintf('processing: %d of %d \n',index, n_sample)
            x_cube_i = [x_voxel(i,j,k);y_voxel(i,j,k);z_voxel(i,j,k)];
            x_i = x_cube_i + voxel_length/2; % middle ofeach voxel
            % init result value
            
            x_all_samples(index,:) = x_i';
            index = index + 1;
        end
    end
end
result{1} = x_all_samples;

% return

%% 
% figure(3)
% grid on
% axis equal

for i = 1:n_sample
    fprintf('processing: %d of %d \n',i, n_sample)

    x_i = result{1}(i,:)';

    % update the current q
%     finger_analysis.update_finger(rand(finger_analysis.nja,1));
    finger_analysis.update_finger(q_limit(:,1)*pi/180);

    [q_i, status_i, ~, x_res,phi_x,iter,q_null_i,phi_x_null_i] = finger_analysis.invkin_trans_numeric_joint_limits_nullspace(...
        x_i,iter_max,tol,alpha,q_diff_min,1);
    
    % ik solutions
    if status_i == 0
        continue
    end
    if status_i >= 1 % only ik solution
        
        assert(max(abs(phi_x(:))) < tol*10, 'nullspace ik error') % tolerance inconsistancy
        
        result{9}(i) = result{9}(i) + 1;
        result{2}{1}(i,:) = q_i';
        finger_analysis.update_finger(q_i);
        
        % calculate the index
        % get Jacobian
        J_index = finger_analysis.Jacobian_analytic_b_end;
        % get reduced Jacobian only with translational terms
        J_index_red = finger_analysis.w_R_base * J_index(1:3,:);
        
        % check the Singularity of Jacobian
        if rank(J_index_red) < 3
            result{9}(i) = result{9}(i) + 1e2;
            continue
        end
        
        % calculate the Mass matrix
        [~,M_r,~,~] = finger_analysis.fordyn_ne_w_end(q_i, zeros(4,1), zeros(4,1), zeros(6,6));
        % calculate the Cartesian acceleration polytope
        P_acc = J_index_red * inv(M_r) * P_tau;
        
        % check condition number of J*M-1
        result{7}{1}(i) = cond(J_index_red * inv(M_r));
        result{8}{1}(i) = cond(J_index_red);
        result{5}{1}(i) = largest_minimum_radius_P_input(P_acc, [0,0,0]');

        % calculate the volume of the acceleration polytope
        try
            result{6}{1}(i) = P_acc.volume;
            if P_acc.volume < 0.001
                % acceleration polytope too small
                result{9}(i) = result{9}(i) + 1e3; % acceleration polytope too small
            elseif P_acc.volume > 100
                % acceleration polytope too large
                result{9}(i) = result{9}(i) + 4e3;
            end
            
        catch
            result{9}(i) = result{9}(i) + 1e4;
        end
        
        % calculate the force polytope
        P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
        
        try
            result{4}{1}(i) = P_ee.volume;
        catch
            result{9}(i) = result{9}(i) + 1e5;
        end
        
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
            if max_pos_f_Ver_i < 0
                result{3}{1}(i,mi) = 0;
            else
                result{3}{1}(i,mi) = max_pos_f_Ver_i;
            end
        end
    end   
    if status_i >= 11
        
        assert(max(abs([phi_x(:);phi_x_null_i(:)])) < tol*10, 'nullspace ik error')
        
        result{9}(i) = result{9}(i) + 4;
        result{2}{2}(i,:) = q_null_i;
        finger_analysis.update_finger(q_null_i);
        
        % calculate the index
        % get Jacobian
        J_index = finger_analysis.Jacobian_analytic_b_end;
        % get reduced Jacobian only with translational terms
        J_index_red = finger_analysis.w_R_base * J_index(1:3,:);
        
        % check the Singularity of Jacobian
        if rank(J_index_red) < 3
            result{9}(i) = result{9}(i) + 1e2;
            continue
        end
        
        % calculate the Mass matrix
        [~,M_r,~,~] = finger_analysis.fordyn_ne_w_end(q_i, zeros(4,1), zeros(4,1), zeros(6,6));
        % calculate the Cartesian acceleration polytope
        P_acc = J_index_red * inv(M_r) * P_tau;
        
        % check condition number of J*M-1
        result{7}{2}(i) = cond(J_index_red * inv(M_r));
        result{8}{2}(i) = cond(J_index_red);
        result{5}{2}(i) = largest_minimum_radius_P_input(P_acc, [0,0,0]');
        % calculate the volume of the acceleration polytope
        try
            result{6}{2}(i) = P_acc.volume;
            if P_acc.volume < 0.001
                % acceleration polytope too small
                result{9}(i) = result{9}(i) + 1e3; % acceleration polytope too small
            elseif P_acc.volume > 100
                % acceleration polytope too large
                result{9}(i) = result{9}(i) + 4e3;
            end
            
        catch
            result{9}(i) = result{9}(i) + 1e4;
        end
        
        % calculate the force polytope
        P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
        
        try
            result{4}{2}(i) = P_ee.volume;
        catch
            result{9}(i) = result{9}(i) + 1e5;
        end
        
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
            if max_pos_f_Ver_i < 0
                result{3}{2}(i,mi) = 0;
            else
                result{3}{2}(i,mi) = max_pos_f_Ver_i;
            end
        end
    end
           
end

% save data
% save('./optimization/results/workspace_1406_x_4mm_30_downwards_index.mat');
% 

return


%% debug
index_i = 16358;

q = opt_act_method.Q(index_i,:)';
finger_analysis.update_finger(q);
figure(100)
finger_analysis.print_finger;
grid on
axis equal
% manipulability index
J_index = finger_analysis.Jacobian_analytic_b_end;
J_index_red = finger_analysis.w_R_base * J_index(1:3,:);
fprintf('cond %f.2 \n', cond(J_index_red))
% svd(J_index_red)
P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
figure(101)
P_ee.plot('alpha',0.5);
hold on

[~,M_r,~,~] = finger_analysis.fordyn_ne_w_end(q, zeros(4,1), zeros(4,1), zeros(6,6));
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



index = 1142887;
q_ori = q_sample_r(index,:)';
x = pos_sample_r(index,:)';
finger_analysis.update_finger(q_ori);
finger_analysis.print_finger('b')

finger_analysis.update_finger([0,0,0,0]');
[q_res, ~, ~,phi_x,~] = finger_analysis.invkin_trans_numeric(x,1000,1e-5,0.1)
finger_analysis.print_finger
finger_analysis.update_finger(finger_analysis.limits_q(:,1))
[q_res2, ~, ~,phi_x,iter_test] = finger_analysis.invkin_trans_numeric_joint_limits(x,1000,1e-9,0.8,1)
finger_analysis.print_finger('c')

hold on 
plot3(x(1),x(2),x(3),'*','MarkerSize',10,'Color','b')
return

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




