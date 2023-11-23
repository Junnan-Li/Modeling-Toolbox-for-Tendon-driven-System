% test script: inverse dynamic (floating base)
%       invdyn_ne_xq_mdh_all_fext.m (floating base)
%       invdyn_ne_mdh_all_fext.m
% 
% Test 1: invdyn_ne_mdh_all_fext.m vs invdyn_ne_xq_mdh_all_fext.m
% Test 2: 
% 
% 
% Junnan Li, junnan.li@tum.de, MIRMI, 02.2023

clear all
close all
clc

%% floating base test
% setting

% number of joints
n_q = 4; % can not change due to the Simscape model

%% random configurations

% random robot states
q = pi*rand(n_q,1);
qD = 5*rand(n_q,1);
qDD = 5*rand(n_q,1);
% gravity acceleration
g = [0 0 -9.81]';

% random robot link mass
Mass = rand(n_q+1,1); % first term is the base mass

% random base position
X_base = rand(6,1);
XD_base = rand(6,1);
XDD_base = rand(6,1);

% zero external force/moment 
F_base = [0,0,0,0,0,0]';
F_end = [0,0,0,0,0,0]';
F_ext_ne_all_fext = [F_base,zeros(6,n_q),F_end];


I = zeros(6,n_q+1); % [xx yy zz yz xz xy]
CoM = zeros(3,n_q+1);
for i = 0:n_q
    CoM(:,i+1) = rand(3,1);
    I(:,i+1) = [rand(3,1);0;0;0]; 
end

% random mdh parameters
% for ne dunction: [alpha,a,theta,d] 
% for rst: [a,alpha,d,theta]
mdh_base = [0,rand(1),0,0];
mdh_ne_link = rand(n_q,4);
mdh_ne_link(:,3) = zeros(n_q,1); % assuming all theta are 0 (changing by q_i)
mdh_ne = [mdh_base;mdh_ne_link];

% Simscape body transformation
W_T_base = [euler2R_XYZ(X_base(4:6)),X_base(1:3);...
                0 0 0 1];
W_R_base = euler2R_XYZ(X_base(4:6));       
ref_T_allframe = zeros(4,4,n_q+1);

for i = 1:n_q+1
    ref_T_allframe(:,:,i) = T_mdh_multi(mdh_ne(i,:));
end

%% invdyn_ne_mdh vs. invdyn_ne_mdh_all_fext.m vs invdyn_ne_xq_mdh_all_fext.m

[Tau_ne_mdh,F_mdh,~] = invdyn_ne_mdh(q,qD,qDD,mdh_ne, Mass,...
    X_base, XD_base, XDD_base, F_end, CoM, I, g);

[Tau_ne_all_fext,F,~] = invdyn_ne_mdh_all_fext(q,qD,qDD,mdh_ne, Mass,...
    X_base, XD_base, XDD_base, F_ext_ne_all_fext, CoM, I, g);

% inverse dynamic with [x_base;q] as input: invdyn_ne_xq_mdh_all_fext.m
[FTau,W_T_allframe] = invdyn_ne_xq_mdh_all_fext([X_base;q],[XD_base;qD],[XDD_base;qDD],mdh_ne,Mass,F_ext_ne_all_fext, CoM, I, g);

FTau_0 = [F_mdh(:,1);Tau_ne_mdh];
FTau_1 = [F(:,1);Tau_ne_all_fext];
FTau_2 = FTau;

% Validation:
test_1_error = FTau_1 - FTau_2;
test_2_error = FTau_1 - FTau_0;
if max(abs(test_1_error(:))) > 1e-8 | max(abs(test_2_error(:))) > 1e-8
    fprintf('Inverse dynamic test 1: failed! \n')
else
    fprintf('Inverse dynamic test 1: pass! \n')
end
return
%% Test 2 forward dynamic 
% fordyn_ne_xq_mdh_fb.m
xq = [X_base;q];
xqD = [XD_base;qD];
[xqDD_fd,M_xq,C_xq,G_xq] = fordyn_ne_xq_mdh_fb(xq,xqD,FTau,mdh_ne, Mass, F_ext_ne_all_fext, CoM, I, g);

% [qDD,M_fd,C_fd,G_fd] = fordyn_ne_mdh(q,qD,FTau(7:end),mdh_ne, Mass, X_base, XD_base,zeros(6,1), F_ext_ne_all_fext_2, CoM, I, g);

% Validation: Inverse dynamic Torque
xqDD_error = [XDD_base;qDD] - xqDD_fd;
% assert(max(M_e(:))<1e-8,'Mass matrix test: failed!')
if max(abs(xqDD_error(:))) > 1e-8
    fprintf('Forward dynamic test all f_ext (NE xq Torque): failed! \n')
else
    fprintf('Forward dynamic test all f_ext (NE xq Torque): pass! \n')
end

%% run Simscape
paramStruct.StopTime = string(1);
out = sim('./test/dynamics/forward_dynamic_Simscape.slx',paramStruct);

%% Validation
% fordyn_ne_xq_mdh_fb.m vs forward_dynamic_Simscape.slx
fprintf('Validation Simscape start: \n')

omega_base = out.omega_base.Data(1,:)';
if max(abs(XD_base(4:6)-omega_base)) < 1e-8
    fprintf('Validation Omega_base: pass! \n')
else
    fprintf('Validation Omega_base: failed! \n')
end

base_link1_rot = out.base_link1_rot.Data(:,:,1);
if max(max(abs(base_link1_rot-ref_T_allframe(1:3,1:3,1))))<1e-8
    fprintf('Validation b_b_R_1: pass! \n')
else
    fprintf('Validation b_b_R_1: failed! \n')
end

link1_2_rot = out.link1_2_rot.Data(:,:,1);
link1_2_rot_ref = T_mdh_multi(mdh_ne(2,:)+[0,0,q(2),0]);
if max(max(abs(link1_2_rot-link1_2_rot_ref(1:3,1:3))))<1e-8
    fprintf('Validation 1_1_R_2: pass! \n')
else
    fprintf('Validation 1_1_R_2: failed! \n')
end


endeff_rot = out.endeffector_rot.Data(:,:,1);
endeff_trans = out.endeffector_trans.Data(1,:)';
w_T_endeff = [endeff_rot,endeff_trans;0,0,0,1];
mdh_ne_q = mdh_ne + [zeros(5,2),[q;0],zeros(5,1)];
endeff_rot_ref = W_T_base*T_mdh_multi(mdh_ne_q);
if max(max(abs(w_T_endeff-endeff_rot_ref)))<1e-8
    fprintf('Validation w_T_endeff: pass! \n')
else
    fprintf('Validation w_T_endeff: failed! \n')
end



qD1 = out.qD1.Data(1,:)';
if max(abs(qD(1)-qD1))<1e-8
    fprintf('Validation qD1: pass! \n')
else
    fprintf('Validation qD1: failed! \n')
end


xqDD_error = xqDD_fd - out.xqDD.Data(1,:)';
if max(abs(xqDD_error))<1e-8
    fprintf('Validation xqDD: pass! \n')
else
    fprintf('Validation xqDD: failed! \n')
end

fprintf('Validation finish!! \n')
