% generate the manipulability maps of the finger objects
% second version


% Junnan Li, junnan.li@tum.de, MIRMI, 05.2023



clear all
% close all
clc


%
% create the index finger and thumb
run create_shadow_hand.m

close all


%% inverse kinematic test
% customized jonit limits

% q_limit_min = finger_little.limits_q(:,1);
% q_limit_max = finger_little.limits_q(:,2);
% %
% [q1,q2,q3,q4,q5] = ndgrid(q_limit_min(1):0.3:q_limit_max(1),...
%         q_limit_min(2):0.3:q_limit_max(2),...
%         q_limit_min(3):0.3:q_limit_max(3),...
%         q_limit_min(4):0.3:q_limit_max(4),...
%         q_limit_min(5):0.3:q_limit_max(5));
% n_sample = size(q1,1)*size(q2,2)*size(q3,3)*size(q4,4)*size(q5,5);
%
% figure(1)
% % finger_index.print_finger()
% % vec_q = [0.2,0.2,0.2,0.2,0]';
% finger_little.update_finger(finger_little.limits_q(:,1));
% finger_little.print_finger('b')
% for i = 1:n_sample
%     q_i = [q1(i),q2(i),q3(i),q4(i),q5(i)]';
%     finger_little.update_finger(q_i);
%     finger_little.print_finger()
% end
% % finger_thumb.print_finger()
% % finger_thumb.update_finger([0,0.2,0,0.4,0.4]');
% % finger_thumb.print_finger('c')
% % finger_thumb.update_finger(finger_thumb.limits_q(:,2));
% % finger_thumb.print_finger('k')
% % finger_index.update_finger([0.35,pi/6,0,0]');
% % finger_index.print_finger()
% % finger_index.update_finger(finger_index.limits_q(:,1));
% % finger_index.print_finger()
% grid on
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
%
% return
%% ik test
% for i = 1:10
%     q = rand(5,1).*(q_limit_max - q_limit_min) + q_limit_min;
%     finger_thumb.update_finger(q);
%     x_all = finger_thumb.get_p_all_links;
%     x = x_all(:,end);
%
%     iter_max = 1000;
%     alpha = 0.7;
%     tol = 1e-8;
%     q_diff_min = 5*pi/180;
%
%
%     finger_thumb.update_finger(q_limit_min);
%     [q_i, status_i, ~, x_res,phi_x,iter,q_null_i,phi_x_null_i] = finger_thumb.invkin_trans_numeric_joint_limits_nullspace(...
%         x,iter_max,tol,alpha,q_diff_min,1);
%     if status_i < 1
%         disp('ik failed')
%     else
%         disp('ik pass')
%     end
% end

%%
% size of the voxel
voxel_length = 0.003;
result_all = struct();

for finger_i = 1:3
    % finger_i: 1 <-- fingers
    %           2 <-- thumb
    %           3 <-- little finger
    
    
    if finger_i == 1 % fingers
        finger_analysis = finger_index;
        % downward (1) or upward 0
        %         direction_downward = 1;
        
        nj = finger_analysis.nja;
        % get coupling matrix
        M_coupling = finger_analysis.M_coupling;
        % get force limits
        force_limits = finger_analysis.limits_ft;
        % generate the torque polytope
        [P_tau, P] = polytope_torque(M_coupling, force_limits);
        P_tau.minHRep;
        P_tau.minVRep;
        
        % customized jonit limits
        q_limit = finger_analysis.limits_q(:,1:2);
        
        % get cartesian workspace cube
        workspace_cube_ori = [0.07,0.23;-0.02,0.09;-0.38,0.18]; % index
        % d_0 setting
        R_d = euler2R_XYZ([0,0,0]);
        
    elseif finger_i == 2 % for thumb
        finger_analysis = finger_thumb;
        % downward (1) or upward 0
        %         direction_downward = 0;
        
        nj = finger_analysis.nja;
        % get coupling matrix
        M_coupling = finger_analysis.M_coupling;
        %         M_coupling = [];
        
        % get force limits
        force_limits = finger_analysis.limits_ft;
        % generate the torque polytope
        [P_tau, P] = polytope_torque(M_coupling, force_limits);
        P_tau.minHRep;
        P_tau.minVRep;
        % customized jonit limits
        q_limit = finger_analysis.limits_q(:,1:2);
        
        % get cartesian workspace cube
        workspace_cube_ori = [-0.01,0.14;-0.02,0.14;-0.08,0.1]; % thumb
        % d_0 setting
        R_d = euler2R_XYZ([-pi*3/4,0,0]);
        
    elseif finger_i == 3 % little finger
        finger_analysis = finger_little;
        % downward (1) or upward 0
        %         direction_downward = 1;
        
        nj = finger_analysis.nja;
        % get coupling matrix
        M_coupling = finger_analysis.M_coupling;
        %         M_coupling = [];
        
        % get force limits
        force_limits = finger_analysis.limits_ft;
        % generate the torque polytope
        [P_tau, P] = polytope_torque(M_coupling, force_limits);
        P_tau.minHRep;
        P_tau.minVRep;
        % customized jonit limits
        q_limit = finger_analysis.limits_q(:,1:2);
        
        % get cartesian workspace cube
        workspace_cube_ori = [0.06,0.23;-0.065,0.045;-0.05,0.15]; % little
        % d_0 setting
        R_d = euler2R_XYZ([pi/4,0,0]);
    end
    % compile mex kinematic/dynamic functions
    finger_analysis.compile_functions;
    
    [x_voxel,y_voxel,z_voxel] = ndgrid(workspace_cube_ori(1,1):voxel_length:workspace_cube_ori(1,2),...
        workspace_cube_ori(2,1):voxel_length:workspace_cube_ori(2,2),...
        workspace_cube_ori(3,1):voxel_length:workspace_cube_ori(3,2));
    
    n_sample = size(x_voxel,1)*size(y_voxel,2)*size(z_voxel,3);
    
    
    %% force direction definition of index finger
    % directions: 9
    % Ae_all [2*5x3], the vectors that are orthogonal to the direction vectors
    % directions: 1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 |
    %             -z|+x-z|-x-z|+y-z|-y-z| +x | -x | +y | -y |
    % d = R * [0,0,-1]
    SQ2 = sqrt(2)/2;
    
    
    Ae_all = (R_d * ...
        [1 0 0;...
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
        ]')';
    
    % 9 direction vectors
    direction_vec = (R_d *...
        [0 0 -1;...
        SQ2 0 -SQ2;...
        -SQ2 0 -SQ2;...
        0 SQ2 -SQ2;...
        0 -SQ2 -SQ2;...
        1 0 0;...
        -1 0 0;...
        0 1 0;...
        0 -1 0]')';
    
    d_0 = R_d * [0,0,-1]';
    
    
    
    %% init variables
    % cell:
    %   1 pos |2 q 2x4 |3 forceindex 5 |4 vol_force |5 acc. r |6 acc.vol |7 conJ |
    %   8 conJM |9 status
    %
    % states: 1: success with nullspace solution
    %         5: success without nullspace solution
    %       10: no ik solution
    %       1e2: Jacobian deficient
    %       1e3: acceleration polytope failed
    %       4e3: acceleration polytope failed
    %       1e4: unable to get acceleration radius
    %       1e5: unable to get force volume
    
    result = {zeros(n_sample,3),{zeros(n_sample,nj),zeros(n_sample,nj)},{zeros(n_sample,9),zeros(n_sample,9)},...
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
            [~,M_r,~,~] = finger_analysis.fordyn_ne_w_end(q_i, zeros(finger_analysis.nja,1), zeros(finger_analysis.nja,1), zeros(6,finger_analysis.nja+2));
            % calculate the Cartesian acceleration polytope
            
            P_acc = Polyhedron('V',(J_index_red * inv(M_r) * P_tau.V')');
            %             P_acc = J_index_red * inv(M_r) * (P_tau); %  - G_fd);
            
            % check condition number of J*M-1
            result{7}{1}(i) = cond(J_index_red * inv(M_r));
            result{8}{1}(i) = cond(J_index_red);
            
            
            % calculate the volume of the acceleration polytope
            try
                P_acc.minVRep;
                P_acc.minHRep;
                result{6}{1}(i) = P_acc.volume;
                result{5}{1}(i) = largest_minimum_radius_P_input(P_acc, [0,0,0]');
                if P_acc.volume < 0.001
                    % acceleration polytope too small
                    result{9}(i) = result{9}(i) + 1e3; % acceleration polytope too small
                elseif P_acc.volume > 1000
                    % acceleration polytope too large
                    result{9}(i) = result{9}(i) + 4e3;
                end
                
            catch
                result{5}{1}(i) = 0;
                result{6}{1}(i) = 0;
                result{9}(i) = result{9}(i) + 1e4;
            end
            
            % calculate the force polytope
            P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
            
            try
                P_ee.minHRep;
                result{4}{1}(i) = P_ee.volume;
            catch
                result{9}(i) = result{9}(i) + 1e5;
            end
            
            % for constrain the half-plan of force polytope
            half_plane_vector = - d_0';
            % calculate the force polytope
            for mi = 1:9 % direction index
                % reduce the dimension from 3 to 1 along the direction
                % vector
                
                %                 half_plane_2 = (-1)^(direction_downward+1) * cross(Ae_all(2*mi-1,:),Ae_all(2*mi,:)); % the correct plane
                p_tmp_force = Polyhedron('A', [P_ee.A;half_plane_vector], 'b',[P_ee.b;0], 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
                p_tmp_force.minHRep;
                Vertex_f_i = p_tmp_force.V;
                if isempty(Vertex_f_i)
                    max_pos_f_Ver_i = 0;
                else
                    max_pos_f_Ver_i = max(diag(sqrt(Vertex_f_i*Vertex_f_i'))); % vextex of the 1-dimension polytope
                end
                result{3}{1}(i,mi) = max_pos_f_Ver_i;
                
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
            [~,M_r,~,~] = finger_analysis.fordyn_ne_w_end(q_i, zeros(finger_analysis.nja,1), zeros(finger_analysis.nja,1), zeros(6,finger_analysis.nja+2));
            % calculate the Cartesian acceleration polytope
            
            %             P_acc = J_index_red * inv(M_r) * (P_tau); %  - G_fd);
            %             P_acc.minHRep;
            P_acc = Polyhedron('V',(J_index_red * inv(M_r) * P_tau.V')');
            
            
            % check condition number of J*M-1
            result{7}{2}(i) = cond(J_index_red * inv(M_r));
            result{8}{2}(i) = cond(J_index_red);
            
            % calculate the volume of the acceleration polytope
            try
                
                P_acc.minVRep;
                P_acc.minHRep;
                result{6}{2}(i) = P_acc.volume;
                result{5}{2}(i) = largest_minimum_radius_P_input(P_acc, [0,0,0]');
                if P_acc.volume < 0.001
                    % acceleration polytope too small
                    result{9}(i) = result{9}(i) + 1e3; % acceleration polytope too small
                elseif P_acc.volume > 100
                    % acceleration polytope too large
                    result{9}(i) = result{9}(i) + 4e3;
                end
                
            catch
                result{9}(i) = result{9}(i) + 5e4;
            end
            
            % calculate the force polytope
            P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
            
            try
                P_ee.minHRep;
                result{4}{2}(i) = P_ee.volume;
            catch
                result{9}(i) = result{9}(i) + 1e5;
            end
            
            % for constrain the half-plan of force polytope
            half_plane_vector = - d_0';
            % calculate the force polytope
            for mi = 1:9 % direction index
                % reduce the dimension from 3 to 1 along the direction
                % vector
                %                 half_plane_2 = (-1)^(direction_downward+1) * cross(Ae_all(2*mi-1,:),Ae_all(2*mi,:));
                p_tmp_force = Polyhedron('A', [P_ee.A;half_plane_vector], 'b',[P_ee.b;0], 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
                p_tmp_force.minHRep;
                p_tmp_force.minVRep;
                Vertex_f_i = p_tmp_force.V;
                if isempty(Vertex_f_i)
                    max_pos_f_Ver_i = 0;
                else
                    max_pos_f_Ver_i = max(diag(sqrt(Vertex_f_i*Vertex_f_i'))); % vextex of the 1-dimension polytope
                end
                result{3}{2}(i,mi) = max_pos_f_Ver_i;
            end
        end
        
    end
    %%
    if finger_i == 1
        result_all.fingers = result;
    elseif finger_i == 2
        result_all.thumb = result;
    elseif finger_i == 3
        result_all.little = result;
    end
    
    
end
% save data
% save('./optimization/results/workspace_3006_x_3mm_30_shadow.mat');

% save './optimization/results/variable_2606_x_3mm_30_shadow_thumb.mat' result result_all


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




