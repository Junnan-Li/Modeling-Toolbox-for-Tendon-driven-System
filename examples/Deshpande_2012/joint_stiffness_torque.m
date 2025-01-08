% joint viscoelastic torque of MCP joint ar different wrist positions
% 
% 
% 
% Reference:
% Deshpande, Ashish D., Nick Gialias, and Yoky Matsuoka. “Contributions of Intrinsic Visco-Elastic Torques 
% during Planar Index Finger and Wrist Movements.” IEEE Transactions on Biomedical Engineering 59, no. 2 
% (2012): 586–94. https://doi.org/10.1109/TBME.2011.2178240.



clear all
close all
clc

%
q = [-60:1:100]';


%% wrist at level position
% M2 subject in the paper
A = 1.27;
B = 0.031;
C = 1.83;
D = 0.07;
E = 47.39;
F = 58.97;
G = 0;
H = 0;

tau_level = double_exp_Deshpande(A,B,C,D,E,F,G,H,q);
%% wrist at extended position
% M2 subject in the paper

G = -31.32;
H = 2.4;

tau_ex = double_exp_Deshpande(A,B,C,D,E,F,G,H,q);
%% wrist at flexed position
% M2 subject in the paper
G = 3.22;
H = 13.18;

tau_flex = double_exp_Deshpande(A,B,C,D,E,F,G,H,q);
figure(1)
grid on
hold on
plot(q,tau_level,'r');
plot(q,tau_ex,'k');
plot(q,tau_flex,'b');


