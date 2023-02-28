% test script: inverse dynamic of serial robots with revolute joints
% comparing results to the robot system toolbox of MATLAB
%
% random joint states, X_base, external force, mass, inertia, com,

clear all
close all
clc
%% setting
n_q = 4;

q = [2;2;3;3];
qD = [1;2;3;1];
qDD = [1;2;3;1];
% gravity acceleration
g = [0 -9.81 0]';

% random robot link mass
% Mass = [0;rand(n_q,1)]; % first term is the base mass
Mass = [1;2;1;4;4];
% random base position
X_base = zeros(6,1);
XD_base = zeros(6,1);
XDD_base = zeros(6,1);

% random external force/moment
F_ext = zeros(6,n_q+2);

% random link inertia of robot with respect to the Center of Mass for ne
% method
I = [ones(6,3),zeros(6,3)]; % [xx yy zz yz xz xy]

% assuming the base frame has the same position with the frame of link 1
CoM = [1,1,0.5,1,1;1,1,0.5,1,1;1,1,0.5,1,1];


% random mdh parameters
% for ne dunction: [alpha,a,theta,d]
% for rst: [a,alpha,d,theta]
mdh_base = zeros(1,4);
mdh_ne_link = rand(n_q,4);
mdh_ne_link(:,3) = zeros(n_q,1); % assuming all theta are 0 (changing by q_i)
mdh_ne = [mdh_base;mdh_ne_link];
mdh_ne = [zeros(n_q+1,1),ones(n_q+1,1),zeros(n_q+1,1),ones(n_q+1,1)];

save('./test/maple/maple_data.mat', 'n_q','Mass','CoM','mdh_ne','I','X_base','XD_base','XDD_base','g')

%% inverse dynamic

% inverse dynamic with [x_base;q] as input: invdyn_ne_xq_mdh_all_fext.m
[FTau,W_T_allframe] = invdyn_ne_xq_mdh_all_fext([X_base;q],[XD_base;qD],[XDD_base;qDD],mdh_ne,Mass,F_ext, CoM, I, g);


% save('./test/dynamics/inv_dyn_example_data.mat','q','qD','qDD','mdh_ne','Mass','X_base','XD_base','XDD_base','F_ext_ne','CoM_ne','I_ne','g');
return
%% Validation results with maple generated code
clc
for i = 1:10
    q = rand(4,1);
    qD = rand(4,1);
    qDD = rand(4,1);
    
    % tau calculation
    tic
    FTau_i = invdyn_ne_xq_mdh_all_fext([X_base;q],[XD_base;qD],[XDD_base;qDD],mdh_ne,Mass,F_ext, CoM, I, g);
    time_mat = toc;
    tau_i = FTau_i(7:end);
    p_i = [q;qD;qDD];
    tic
    tau = invdyn_maple_testconfig(p_i);
    time_maple = toc;
    
    tau_error = abs(tau_i-tau);
    %     fprintf('Matlab time cost: %f! \n',time_mat)
    %     fprintf('Maple time cost: %f! \n',time_maple)
    
    % mass matric calculation
    tic
    M_matrix = invdyn_maple_testconfig_M(p_i);
    time_maple_M = toc;
    
    tic
    M_xq = Mass_Gravity_ne_xq_mdh_fb([X_base;q],mdh_ne, Mass, F_ext, CoM, I, g);
    time_mat_M = toc;
    M_xq_q = M_xq(7:end,7:end);
    M_error = abs(M_xq_q-M_matrix);
    
    if max(M_error(:)) > 1e-10 && max(tau_error(:)) > 1e-10
        fprintf('iteration %d: failed! \n',i)
    else
        fprintf('iteration %d: pass! \n',i)
    end
    
    fprintf('Tau computational improvement: %f \n', (time_mat-time_maple)/time_mat)
    fprintf('M computational improvement: %f \n', (time_mat_M-time_maple_M)/time_mat_M)
end



