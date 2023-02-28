


%% create finger with tendons

clear all 
close all
clc

% define a finger
finger_dimension = [0.05,0.03,0.02]; % in meter
finger_r = Finger('Index', 'RRRR', finger_dimension); 

%% set states
% set base position and orientation
finger_r.w_p_base = 4*zeros(3,1);
finger_r.w_R_base = euler2R_XYZ(zeros(1,3));

% init joint configurations
q_0 = [0;45;45;10]*pi/180;

% udpate finger with given joint configurations
finger_r.update_finger(q_0);
% load rst model from finger class
rst_model = finger_r.rst_model;

%% set dynamic parameters
% link index:
%   1: PP
%   2: MP
%   3: DP

finger_r.list_links(1).set_mass(0.05); % in kg
finger_r.list_links(1).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(2).set_mass(0.03); % in kg
finger_r.list_links(2).set_inertia([0.5,0.2,0.2,0,0,0]); 

finger_r.list_links(3).set_mass(0.03); % in kg
finger_r.list_links(3).set_inertia([0.5,0.2,0.2,0,0,0]); 

% update dynamic parameters
finger_r.update_finger_par_dyn;
% finger_r.update_finger(q_0);

%% add tendons
if finger_r.nt == 0
    finger_r.add_tendon('Flex_1', [1,1,1,1]);
    finger_r.add_tendon('Flex_2', [1,1,1,0]);
    finger_r.add_tendon('Flex_3', [1,1,0,0]);
    finger_r.add_tendon('Flex_4', [1,0,0,0]);
    finger_r.add_tendon('Ext_1', [-1,-1,-1,-1]);
    finger_r.add_tendon('Ext_2', [-1,-1,-1,0]);
    finger_r.add_tendon('Ext_3', [-1,-1,0,0]);
    finger_r.add_tendon('Ext_4', [-1,0,0,0]);
end

finger_r.update_finger(q_0);
finger_r.update_M_coupling(q_0);

for i = 1:finger_r.nt
    tendon_nj = size(finger_r.list_tendons(i).par_MA_poly3,2);
    for j = 1:tendon_nj
        finger_r.set_tendon_par_MA_poly3(i,j,[0,0,0,0.015]);
    end
end

finger_r.update_finger(q_0);
finger_r.update_M_coupling(q_0);


% set joint limits
finger_r.list_joints(1).q_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qd_limits = [-15,15]*pi/180; % abduction joints
finger_r.list_joints(1).qdd_limits = [-15,15]*pi/180; % abduction joints

finger_r.update_joints_info;


%% add contacts
if finger_r.nc == 0
    for i = 1:finger_r.nl
        finger_r.list_links(i).add_contact([finger_r.list_links(i).Length/2 0 0]');
    end
end
finger_r.update_list_contacts; % update link


%% optimization setting
opt_iter = 5;
opt_routing = 1;

opt_result = {};

% [LMS_r, LMS_x, origin_included] = metric_torque_LMS(C, force_limits);
C = finger_r.M_coupling;

opt_var = [];
opt_var_index = [];
opt_var_limits = [];
opt_ft_limit = finger_r.limits_ft;

if opt_routing
    for i = 1:finger_r.nt
        tendon_nj = size(finger_r.list_tendons(i).par_MA_poly3,2);
        for j = 1:tendon_nj
            p_var_tendon_i_j_routing = finger_r.list_tendons(i).routing(j);
            opt_var = [opt_var;p_var_tendon_i_j_routing];
            opt_var_index = [opt_var_index;[j,i]];
            opt_var_limits = [opt_var_limits;[-1,1]];
        end
    end
end

for i = 1:finger_r.nt
    tendon_nj = size(finger_r.list_tendons(i).par_MA_poly3,2);
    for j = 1:tendon_nj
        p_var_tendon_i_j = C(j,i);
        opt_var = [opt_var;p_var_tendon_i_j];
        opt_var_index = [opt_var_index;[j,i]];
        opt_var_limits_i_j = sort(finger_r.limits_t_ma(i,2*j-1:2*j));
        opt_var_limits = [opt_var_limits;opt_var_limits_i_j];
    end
end

% get Jacobian of endeffector
b_J_end = finger_r.Jacobian_geom_b_end(q_0);
w_J_end = blkdiag(finger_r.w_R_base,finger_r.w_R_base) * b_J_end;
w_J_end_red = w_J_end(1:3,:);


%% optimization

opt_fitness_optinos.opt_routing = opt_routing;
fitnessfcn_torque_LMS = @(opt_var)fitness_torque_LMS(opt_var,opt_var_index,opt_ft_limit,opt_fitness_optinos);
fitnessfcn_ft_force_LMS = @(opt_var)fitness_fingertip_LMS(opt_var,opt_var_index,opt_ft_limit,w_J_end_red,opt_fitness_optinos);

options = optimoptions('particleswarm','MaxIterations',100,'Display','iter','SwarmSize',300);

fprintf('optimization start! \n')
% parfor i = 1:opt_iter
% %     tic;
%     [x,fval,exitflag,output] = particleswarm(fitnessfcn_ft_force_LMS,length(opt_var),opt_var_limits(:,1),opt_var_limits(:,2),options);
% %     time_cost = toc;
% %     fprintf('optimization iteration %e: %f \n',i,time_cost);
%     opt_result{i}.x = x;
%     opt_result{i}.fval = fval;
%     opt_result{i}.exitflag = exitflag;
%     opt_result{i}.output = output;
% end


% save('./optimization/results/result0223.mat','opt_result')

return
%% plot results
load('./optimization/results/result0220.mat')

h = figure;
set(0,'defaultfigurecolor','w')
set(h,'units','normalized','outerposition',[0 0 1 1])
set(h, 'Units', 'centimeters')
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)]);%

set(gca,'FontSize', 25)
set(gcf, 'Units', 'centimeters')
set(gca,'FontSize',25,'FontName','Arial')
a = get(gca,'XTickLabel');
set(gca,'XTickLabel',a,'FontName','Arial','fontsize',25)
    
nvar = length(opt_var);
C = [];
for result_index = 1:5
    if opt_fitness_optinos.opt_routing
        for i = 1:nvar/2
            C(opt_var_index(nvar/2+i,1),opt_var_index(nvar/2+i,2)) = sign(opt_result{result_index}.x(i))*opt_result{result_index}.x(nvar/2+i);
        end
    else
        for i = 1:nvar
            C(opt_var_index(i,1),opt_var_index(i,2)) = opt_result{result_index}.x(i);
        end
    end
    C
    LMS_r = metric_torque_LMS(C, opt_ft_limit)
    % LMS_r = metric_torque_LMS(finger_r.M_coupling, opt_ft_limit)
    LMS_r = metric_fingertip_force_LMS(C, w_J_end_red, opt_ft_limit)
    % LMS_r = metric_fingertip_force_LMS(finger_r.M_coupling, w_J_end_red, opt_ft_limit)
    
    % return
    % plot
    P_ft = polytope_fingertip_3d(C, w_J_end_red, opt_ft_limit);
    subplot(2,3,result_index)
    P_ft.plot('alpha',0.2,'color','r');
    hold on
    % plot sphere
    r = LMS_r;
    [X,Y,Z] = sphere;
    X2 = X * r;
    Y2 = Y * r;
    Z2 = Z * r;
    surf(X2,Y2,Z2)
    hold on
    P_ft_ori = polytope_fingertip_3d(0.2/0.15*finger_r.M_coupling, w_J_end_red, opt_ft_limit);
    P_ft_ori.plot('alpha',0.2,'color','b');
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')

end



