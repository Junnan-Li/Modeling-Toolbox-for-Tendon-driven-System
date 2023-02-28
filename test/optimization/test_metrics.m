%   test scripts for all metrics transfered from the previous repo

%   TODO: 
%       1. joint dimension increases to 4. not visualization
%       2. experimental results is planar. metrics at fingertip force level
%       need to be modified
%       3. 
%       

run create_hand.m

% finger object: finger_r
q = [0;10;10;10]*pi/180;

finger_r.update_finger(q);


% show(finger_r.rst_model,q,'Collisions','on','Visuals','off')
figure(1)
finger_r.print_contact
grid on
axis equal


% get the info of finger
nj = finger_r.nj;
nt = finger_r.nt;
C = finger_r.M_coupling; % Coupling matrix

% get Jacobian of endeffector
b_J_end = finger_r.Jacobian_geom_b_end(q);
w_J_end = blkdiag(finger_r.w_R_base,finger_r.w_R_base) * b_J_end;
w_J_end_red = w_J_end(1:3,:); % only force term 

% tendon force limits
force_limits = finger_r.limits_ft;




P_tau = polytope_torque(C, force_limits);
P_ft = polytope_fingertip_3d(C, w_J_end_red, force_limits);

%% tendon space algebraic analysis
[sigma_min,sigma_vol,sigma_isotropic] = metric_algebraic_analysis(C);

fprintf('========================================== \n')
%% torque space calculate radius of Largest-Minimum-Sphere
[LMS_r, LMS_x, origin_included] = metric_torque_LMS(C, force_limits);

fprintf('Torque analysis: radius of LMS: %.6e \n',LMS_r )
% fprintf('Torque analysis: position of LMS: %.6e \n',LMS_x )
fprintf('Torque analysis: origin included: %d \n', origin_included )
fprintf('========================================== \n')
%% fingertip space
% argebraic analysis

pinv_w_J_end_T = pinv(w_J_end_red');
pinv_w_J_end_T_C = pinv_w_J_end_T * C;

[sigma_min,sigma_vol,sigma_isotropic] = metric_algebraic_analysis(pinv_w_J_end_T_C);

% polytope of human:

a1 = 40.9;
b1 = 37.6;
a2 = 12.8;
b2 = b1;
area_human = pi*(a1*b1+a2*b2)/4;


fprintf('========================================== \n')
return

%%
poly_color = {[0.3010 0.7450 0.9330], [0.9290 0.6940 0.1250]};

h = figure;
% set(0,'defaultfigurecolor','w') 
 P_tau.plot('alpha',0.4,'color',poly_color{1},'edgealpha',0.6) % 'wire',true
set(0,'defaultfigurecolor','w') 
set(h,'Units','normalized','outerposition',[0 0 1 1])
set(h, 'Units', 'centimeters')
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)]);%
set(h, 'Units', 'centimeters')
% set(gca,'Units','centimeters')
axis equal
hold on
plot(0,0,'.','MarkerSize',60,'Color','r')
hold on
% plot Largest-Minimum-Sphere
[x_sph,y_sph,z_sph] = sphere;
x_sph = x_sph*LMS_r;
y_sph = y_sph*LMS_r;
z_sph = z_sph*LMS_r;
surf(x_sph,y_sph,z_sph,'FaceAlpha',0.4,'FaceColor','g','EdgeAlpha',0.2)
grid on


return

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
P_ft.plot('alpha',0.2);
axis equal
xlabel('x')
ylabel('y')
zlabel('z')











