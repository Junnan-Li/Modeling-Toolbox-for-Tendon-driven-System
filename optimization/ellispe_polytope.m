% draw ellipse and polytope
% 
% 
%   Junnan Li, junnan.li@tum.de, MIRMI, 02.2023



clear all
close all
clc

%%

% J = rand(2,2);
J = [0.6,0.4;0.2,0.6];

[U,S,V] = svd(J);

%%
% inequalities for constraints of torque
H_1 = [eye(2);-eye(2)];
B_1 = [[1;1]; [1;1]];


% mpt toolbox define a polytope
P_input_poly = Polyhedron('A', H_1, 'b', B_1);

% P_input_poly.plot

P_res = J * P_input_poly;
% P_res.plot
%%

t = linspace(0,2*pi);
x_input = cos(t);
y_input = sin(t);


XY = J * [x_input;y_input];


%%

h = figure(15);
set(0,'defaultfigurecolor','w')
set(groot,'defaulttextinterpreter','none');
% set(h,'units','normalized','outerposition',[0 0 1 1])
set(h, 'Units', 'centimeters')
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)]);%
set(h, 'Units', 'centimeters')
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');



subplot(1,2,1)
plot(x_input,y_input,'r-','LineWidth',4)
hold on
P_input_poly.plot('alpha',0,'LineWidth',4,'edgecolor','b')
axis equal
grid off
set(gca,'xticklabel',[],'xtick',[])
set(gca,'yticklabel',[],'ytick',[])
set(gca,'Visible','off')

subplot(1,2,2)
plot(XY(1,:)',XY(2,:)' ,'r-','LineWidth',4)
hold on
P_res.plot('alpha',0,'LineWidth',4,'edgecolor','b')
axis equal
grid off
set(gca,'xticklabel',[],'xtick',[])
set(gca,'yticklabel',[],'ytick',[])
set(gca,'Visible','off')
