






clear all
close all
clc



run opt_create_thumb_index.m

close all
% test poses
% 
q = [-15;0;0;0]*pi/180;
q_t = [0;0;0;0]*pi/180;
finger_r.update_finger(q);
finger_t.update_finger(q_t);
finger_r.update_list_contacts;
figure(2)
finger_r.print_finger
% finger_t.print_finger
q = [15;0;0;0]*pi/180;
finger_r.update_finger(q);
finger_r.print_finger
grid on
axis equal

xlabel('x')
ylabel('y')
zlabel('z')


% return

%% pre-defined direction

nj = finger_r.nja;
% A = [eye(4);-1*eye(4)];
% b = ones(2*nj,1);
% P_tau = Polyhedron('A', A, 'b', b);

M_coupling = finger_r.M_coupling;
force_limits = finger_r.limits_ft;

[P_tau, P] = polytope_torque(M_coupling, force_limits);

% directions: 5
% Ae_all [2*5x3]
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

direction_vec = [0 0 -1;...
    SQ2 0 -SQ2;...
    -SQ2 0 -SQ2;...
    0 SQ2 -SQ2;...
    0 -SQ2 -SQ2];

q_limit = [-10,10;0,90;0,80;0,40];

[q1,q2,q3,q4] = ndgrid(q_limit(1,1):5:q_limit(1,2),...
    q_limit(2,1):10:q_limit(2,2),...
    q_limit(3,1):10:q_limit(3,2),...
    q_limit(4,1):10:q_limit(4,2));

n_sample = size(q1,1)*size(q1,2)*size(q1,3)*size(q1,4);


% finger_r
mani_index_r = zeros(n_sample,5);
mani_index_r_acc = zeros(n_sample,5);
pos_sample_r = zeros(n_sample,3);
q_sample_r = zeros(n_sample,4);
vol_sample_r = zeros(n_sample,1);
vol_sample_r_acc = zeros(n_sample,1);
index = 1;

opt_act_method = struct();
opt_act_method.Metric_force = zeros(n_sample,5);
opt_act_method.Metric_acc = zeros(n_sample,5);
opt_act_method.end_pos = zeros(n_sample,3);
opt_act_method.Q = zeros(n_sample,4);
opt_act_method.vol_force = zeros(n_sample,1);
opt_act_method.vol_acc = zeros(n_sample,1);
% 

figure(3)
grid on
axis equal
for i = 1:size(q1,1)
    for j = 1: size(q1,2)
        for k = 1:size(q1,3)
            for h = 1:size(q1,4)
                
                q_i = [q1(i,j,k,h);q2(i,j,k,h);q3(i,j,k,h);q4(i,j,k,h)]*pi/180;
                finger_r.update_finger(q_i); 

                finger_r.print_finger;
                hold off
                drawnow

                pos_i = finger_r.get_p_all_links;
                pos_sample_r(index,:) = pos_i(:,end)';
                q_sample_r(index,:) = q_i';
                % manipulability index
                J_index = finger_r.Jacobian_analytic_b_end;
                J_index_red = finger_r.w_R_base * J_index(1:3,:);
                
                if rank(J_index_red) < 3
                    mani_index_r(index,:) = -0*ones(1,5);
                    index = index + 1;
                    continue
                end
                [~,M_r,~,~] = finger_r.fordyn_ne_w_end(q_i, zeros(4,1), zeros(4,1), zeros(6,6));
                P_acc = J_index_red* inv(M_r)*P_tau;
                
                try
                    vol_sample_r_acc(index) = P_acc.volume;
                catch
                    %                     warning('Problem using function.  Assigning a value of 0.');
                    vol_sample_r_acc(index) = 0;
                end

                P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
                
                try
                    vol_sample_r(index) = P_ee.volume;
                catch
                    %                     warning('Problem using function.  Assigning a value of 0.');
                    vol_sample_r(index) = 0;
                end

                for mi = 1:5 
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
%                 test(10,J_index_red,P_ee,P_tau,p_tmp,finger_r); % test use
                index = index + 1;
                fprintf('processing: %d of %d \n',index, n_sample)
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

% % finger_t
% mani_index_t = zeros(n_sample,5);
% pos_sample_t = zeros(n_sample,3);
% 
% index = 1;
% M_coupling = finger_t.M_coupling;
% force_limits = finger_t.limits_ft;
% 
% for i = 1:size(q1,1)
%     for j = 1:size(q1,2)
%         for k = 1:size(q1,3)
%             for h = 1:size(q1,4)
%                 q_i = [q1(i,j,k,h);q2(i,j,k,h);q3(i,j,k,h);q4(i,j,k,h)]*pi/180;
%                 finger_t.update_finger(q_i);                
%                 pos_i = finger_t.get_p_all_links;
%                 pos_sample_t(index,:) = pos_i(:,end)';
%                 % manipulability index
%                 J_index = finger_t.Jacobian_analytic_b_end;
%                 J_index_red = finger_t.w_R_base * J_index(1:3,:);
%                 if rank(J_index_red) < 3
%                     mani_index_t(index,:) = 0*ones(1,5);
%                     index = index + 1;
%                     continue
%                 end
%                 [~,M_r,~,~] = finger_t.fordyn_ne_w_end(q_i, zeros(4,1), zeros(4,1), zeros(6,6));
%                 P_ee = J_index_red * inv(M_r)*P_tau;
%                 
% %                 P_ee = polytope_fingertip_3d(M_coupling, J_index_red, force_limits);
% 
%                 for mi = 1:5
%                     p_tmp = Polyhedron('A', P_ee.A, 'b', P_ee.b, 'Ae',Ae_all(2*mi-1:2*mi,:) ,'be', zeros(2,1));
%                     vec_tmp = p_tmp.distance(-1000*direction_vec(mi,:)').y;
%                     dist_tmp = p_tmp.distance(-1000*direction_vec(mi,:)').dist;
%                     if dist_tmp >= 1000
%                         mani_index_t(index,mi) = 0;
%                     else
%                         mani_index_t(index,mi) = sqrt(vec_tmp'*vec_tmp);
%                     end
%                 end
%                 index = index + 1;
%             end
%          end
%     end
% end

% % save data
% save('./test/optimization/index_0805.mat',"opt_act_method");


return
%%
index_i = 92;

q = opt_act_method.Q(index_i,:)';
finger_r.update_finger(q);
figure(100)
finger_r.print_finger;
grid on
axis equal
% manipulability index
J_index = finger_r.Jacobian_analytic_b_end;
J_index_red = finger_r.w_R_base * J_index(1:3,:);
fprintf('cond %f.2 \n', cond(J_index_red))
% svd(J_index_red)
P_ee = polytope_fingertip_3d(M_coupling, J_index_red,force_limits); % force polytope but not acceleration
figure(101)
P_ee.plot('alpha',0.5);
hold on

[~,M_r,~,~] = finger_r.fordyn_ne_w_end(q, zeros(4,1), zeros(4,1), zeros(6,6));
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

%% 
% index_max = max(mani_index(:));
% weight = [2 1 1 1 1];
% weighted_index_r = mani_index_r * weight';
% index_max_r = max(weighted_index_r);
% index_min_r = min(weighted_index_r);

% weighted_index_t = mani_index_t * weight';
% index_max_t = max(weighted_index_t);
% index_min_t = min(weighted_index_t);
% 

% q = [0;0;0;0]*pi/180;
% q_t = [0;0;0;0]*pi/180;
% finger_r.update_finger(q);
% finger_t.update_finger(q_t);


mani_index_plot = mani_index_r;

% finger_r.print_finger('r')
% finger_t.print_finger('g')

figure(10)
for j = 1:5
    index_max_r = max(mani_index_plot(:,j));
    index_min_r = min(mani_index_plot(:,j));
    
%     index_max_t = max(mani_index_t(:,j));
%     index_min_t = min(mani_index_t(:,j));

    subplot(2,3,j)
    for i = 1:n_sample
        p_link_all_w_r = pos_sample_r(i,:);
        plot3(p_link_all_w_r(1),p_link_all_w_r(2),p_link_all_w_r(3),'.',...
            'Color',[1 0 0]*(mani_index_plot(i,j)-index_min_r)/(index_max_r-index_min_r));
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




%% 
% 

function status = test(i,J_index_red,P_ee,P_tau,p_tmp,finger_r)
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
finger_r.print_finger;
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







