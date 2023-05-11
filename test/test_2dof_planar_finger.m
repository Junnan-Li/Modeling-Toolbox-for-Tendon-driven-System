


clear all
close all 
clc






%% [alpha, a theta, d]


mdh_parameter = [0 0 0 0;...
                  0 1 0 0;...
                  0 1 0 0];

mdh_struct = mdh_matrix_to_struct(mdh_parameter, 1);
finger_r = Finger('Index', 'mdh',mdh_struct );


q = [35,45]*pi/180;
finger_r.update_finger(q);


J_a = finger_r.Jacobian_analytic_b_end;
J_g = finger_r.Jacobian_geom_b_end;


J_g_red2 = J_g([1,2],:);

[P_tau2, P] = polytope_torque(J_g_red2, [-1,1;-1,1]);

pos_links = finger_r.get_p_all_links;


figure(3)
plot(pos_links(1,:), pos_links(2,:), '-', 'Color','r','LineWidth',5);
hold on
grid on
axis equal
P_tau2.plot
hold on

%% plot motion ellipse

[U,S,V] = svd(J_g_red2);
% Define the ellipse parameters
a = S(1,1); % Length of semi-major axis
b = S(2,2); % Length of semi-minor axis
xc = pos_links(1,3); % x-coordinate of center
yc = pos_links(2,3); % y-coordinate of center
theta = atan(U(2,1)/U(1,1)); % Angle between x-axis and semi-major axis

% Generate points on the ellipse
t = linspace(0, 2*pi, 100);
x = xc + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y = yc + a*cos(t)*sin(theta) + b*sin(t)*cos(theta);


% Plot the ellipse
% plot(x, y, 'Color','b','LineWidth',2);
% axis equal; % Set equal aspect ratio for x and y axis
% hold on;


%% plot force ellipse

[U,S,V] = svd(inv(J_g_red2'));
% Define the ellipse parameters
a = S(1,1); % Length of semi-major axis
b = S(2,2); % Length of semi-minor axis
theta = atan(U(2,1)/U(1,1)); % Angle between x-axis and semi-major axis

% Generate points on the ellipse
t = linspace(0, 2*pi, 100);
x = xc + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y = yc + a*cos(t)*sin(theta) + b*sin(t)*cos(theta);


% Plot the ellipse
plot(x, y, 'Color','m','LineWidth',2);
axis equal; % Set equal aspect ratio for x and y axis
hold on;

% Add labels
xlabel('x');
ylabel('y');
title('Mannipulability of motion and force');
legend('planar robot', 'motion ellipse', 'force elipse')

%% polytope Norm plot

H_1 = [eye(2);-eye(2)];
B_1 = [[1;1]; [1;1]];
He_1 = [];
Be_1 = [];
% mpt toolbox define a polytope
P = Polyhedron('A', H_1, 'b', B_1, 'Ae', He_1, 'be', Be_1);

figure(4)
P.plot('alpha', 0.5)
hold on

a = 1; % Length of semi-major axis
b = 1; % Length of semi-minor axis
theta = 0; % Angle between x-axis and semi-major axis

% Generate points on the ellipse
t = linspace(0, 2*pi, 100);
x = a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y = a*cos(t)*sin(theta) + b*sin(t)*cos(theta);

% Plot the ellipse
plot(x, y, 'Color','b','LineWidth',3);
axis equal; % Set equal aspect ratio for x and y axis
hold on;

xlim([-2,2])
ylim([-2,2])




%% polytope plot

[P_tau2, P] = polytope_torque(inv(J_g_red2'), [-1,1;-1,1]);

figure(5)


P_tau2.plot
hold on
grid on
axis equal

[U,S,V] = svd(inv(J_g_red2'));
% Define the ellipse parameters
a = S(1,1); % Length of semi-major axis
b = S(2,2); % Length of semi-minor axis
theta = atan(U(2,1)/U(1,1));
% Generate points on the ellipse
t = linspace(0, 2*pi, 100);
x = a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y = a*cos(t)*sin(theta) + b*sin(t)*cos(theta);

% Plot the ellipse
plot(x, y, 'Color','b','LineWidth',3);
axis equal; % Set equal aspect ratio for x and y axis
hold on;


