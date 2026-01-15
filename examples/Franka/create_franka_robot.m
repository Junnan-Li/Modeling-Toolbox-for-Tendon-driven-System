% create a Franka robot 
% parameters are derived from peter corke toolbox
% https://github.com/petercorke/robotics-toolbox-matlab/blob/master/models/mdl_panda.m
% 
% 
% need to 

clear all
close all
clc
%% Robot Definition from peter corke matlab toolbox
deg = pi/180;
mm = 1e-3;

% Define links from: https://github.com/petercorke/robotics-toolbox-matlab/blob/master/models/mdl_panda.m
L1 = RevoluteMDH('a',     0.0, 'd', 0.333, 'alpha',   0.0, 'qlim', [-2.8973 2.8973], ...
    'm', 4.970684, 'r', [3.875e-03 2.081e-03 0], 'I', [7.03370e-01  7.06610e-01  9.11700e-03 -1.39000e-04  1.91690e-02  6.77200e-03], 'G', 1);
L2 = RevoluteMDH('a',     0.0, 'd',   0.0, 'alpha', -pi/2, 'qlim', [-1.7628 1.7628], ...
    'm', 0.646926, 'r', [-3.141e-03 -2.872e-02 3.495e-03], 'I', [7.96200e-03  2.81100e-02  2.59950e-02 -3.92500e-03  7.04000e-04  1.02540e-02], 'G', 1);
L3 = RevoluteMDH('a',     0.0, 'd', 0.316, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], ...
    'm', 3.228604, 'r', [ 2.7518e-02 3.9252e-02 -6.6502e-02], 'I', [3.72420e-02  3.61550e-02  1.08300e-02 -4.76100e-03 -1.28050e-02 -1.13960e-02], 'G', 1);
L4 = RevoluteMDH('a',  0.0825, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-3.0718 -0.0698], ...
    'm', 3.587895, 'r', [-5.317e-02 1.04419e-01 2.7454e-02], 'I', [2.58530e-02  1.95520e-02  2.83230e-02  7.79600e-03  8.64100e-03 -1.33200e-03], 'G', 1);
L5 = RevoluteMDH('a', -0.0825, 'd', 0.384, 'alpha', -pi/2, 'qlim', [-2.8973 2.8973], ...
    'm', 1.225946, 'r', [-1.1953e-02 4.1065e-02 -3.8437e-02], 'I', [3.55490e-02  2.94740e-02  8.62700e-03 -2.11700e-03  2.29000e-04 -4.03700e-03], 'G', 1);
L6 = RevoluteMDH('a',     0.0, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-0.0175 3.7525], ...
    'm', 1.666555, 'r', [6.0149e-02 -1.4117e-02 -1.0517e-02], 'I', [1.96400e-03  4.35400e-03  5.43300e-03  1.09000e-04  3.41000e-04 -1.15800e-03], 'G', 1);
L7 = RevoluteMDH('a',   0.088, 'd',   0.107, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], ...
    'm', 7.35522e-01, 'r', [1.0517e-02 -4.252e-03 6.1597e-02 ], 'I', [1.25160e-02  1.00270e-02  4.81500e-03 -4.28000e-04 -7.41000e-04 -1.19600e-03], 'G', 1);

% Create SerialLink object 
panda = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'PANDA', 'manufacturer', 'Franka-Emika', 'tool', transl([0 0 110*mm]));
panda.gravity = [0, 0, -9.80665];


% panda = loadrobot("frankaEmikaPanda");
%% create franka robot
franka = create_finger_random('Franka', 7);

mdh_default_struct = franka.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix = [0,     0,   0,0.333;...
              -pi/2, 0,   0, 0;...
              pi/2,  0,   0, 0.316;...
              pi/2,  0.0825, 0, 0; ...
              -pi/2, -0.0825, 0, 0.384;...
              pi/2,  0,   0, 0;...
              pi/2,  0.088, 0, 0.107;
              0,     0,     0, 0.11];


franka.set_mdh_parameters(mdh_matrix);
franka.set_base([0,0,0]',eye(3))

plot_par = franka.plot_parameter_init();
plot_par.axis_len = 0.3;
for i = 1:7
    franka.list_links(i).set_mass((panda.links(i).m));
    franka.list_links(i).set_com(panda.links(i).r);

    Ixx = panda.links(i).I(1,1);
    Iyy = panda.links(i).I(2,2);
    Izz = panda.links(i).I(3,3);
    Ixy = panda.links(i).I(1,2);
    Iyz = panda.links(i).I(2,3);
    Ixz = panda.links(i).I(1,3);
    % in sequence of [xx yy zz yz xz xy]
    franka.list_links(i).set_inertia([Ixx Iyy Izz Iyz Ixz Ixy]);
end

franka.update_finger_par_dyn;
franka.set_g_w(panda.gravity);

% testing kinematic and dynamic 
cal_error = 0;
visual_kin = 0;
for samples = 1:100
    q_init = rand(1,7);
    franka.update_finger(q_init');
    if visual_kin
        figure(10)
        subplot(1,2,1)
        franka.plot_finger(plot_par)
        grid on
        axis equal
        xlim([-0.8 0.8])
        ylim([-0.8 0.8])
        zlim([0 1.3])
        axis manual
        drawnow
        hold off
        subplot(1,2,2)
        panda.plot(q_init,'jointdiam',1)
        axis equal
        xlim([-0.8 0.8])
        ylim([-0.8 0.8])
        zlim([0 1.3])
        axis manual
        drawnow
        hold off
    end
    [Ts,T_all] = panda.fkine(q_init);
    T_ee = franka.get_T_all_links;
    error_T = [];
    for i = 1:7
        t_pc_i = T_all(i).T;
        T_link_i = T_ee(:,:,i+1);
        error_T = [error_T;t_pc_i-T_link_i];
    end
    if max(abs(error_T(:))) > 1e-6
        cal_error = 1;
        break;
    end
end
if cal_error
    fprintf('Panda kinematic computation testing:  ERROR! \n')
else
    fprintf('Panda kinematic computation testing:  SUCCESS! \n');
end


%% test symbolic functions

t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;
% cal_dyn_error = 0;
num_samples = 100;
for samples = 1:num_samplesd
    q = rand(1,7);
    qd = rand(1,7);

    % % dynamic torque from Peter Corke toolbox
    % M_peter = panda.inertia(q);
    % G_peter = panda.gravload(q);
    % C = panda.coriolis(q, qd) * qd';
    
    % dynamic torque from hand toolbox
    [qDD,M_fd,C_fd,G_fd] = franka.fordyn_ne_w_end(q', qd',zeros(franka.nj,1), zeros(6,franka.nj+2));

    % compare results
    tic
    M_sym = invdyn_lag_mdh_sym_Franka_M(q');
    t1 = t1 + toc;
    tic
    M_sym2 = M_Franka(q');
    t2 = t2 + toc;

    tic
    G_1 = invdyn_lag_mdh_sym_Franka_G(q');
    t3 = t3 + toc;

    tic
    Tau = invdyn_lag_mdh_sym_Franka_Tau(q',qd');
    C2 = Tau-G_1;
    t4 = t4 + toc;

    % error_M1 = M_fd-M_sym;
    % error_M2 = M_fd-M_sym2;
    % error_M_P = M_fd - M_peter; % error of hand toolbox and Peter
    % error_G = G_fd- G_1;
    % error_C = C_fd - C2;
    % error_all = [error_M1,error_M2,error_M_P,error_G,error_C];
    % if max(abs(error_all(:))) > 1e-9
    %     cal_dyn_error = 1;
    %     break
    % end
end
% if cal_dyn_error
%     fprintf('Panda dynamic computation testing:  ERROR! \n')
% else
%     fprintf('Panda dynamic computation testing:  SUCCESS! \n');
% end
fprintf('Panda dynamic computation time consumption: \n');
fprintf('invdyn_lag_mdh_sym_Franka_M.m :   %f \n', t1/num_samples)
fprintf('                   M_Franka.m :   %f \n', t2/num_samples)
fprintf('invdyn_lag_mdh_sym_Franka_G.m :   %f \n', t3/num_samples)
fprintf('invdyn_lag_mdh_sym_Franka_C.m :   %f \n', t4/num_samples)
return

%% code used to generat2 symbolic functions using Lagrangian 

q_r_sym = sym('q',[franka.nj,1], 'real');
q_rd_sym = sym('qd',[franka.nj,1], 'real');
mdh_default_struct = franka.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
x_base = zeros(6,1);
var_name = {q_r_sym,q_rd_sym};

% tau with q and qd (for C)
[Tau] = invdyn_ne_mdh_sym(q_r_sym,q_rd_sym,sym(zeros(franka.nj,1)),...
                 sym(mdh_matrix), sym(franka.par_dyn_f.mass_all),...
            sym(x_base), sym(x_base),sym(x_base), sym(zeros(6,1)),...
            sym(franka.par_dyn_f.com_all),sym(franka.par_dyn_f.inertia_all),...
            sym(franka.par_dyn_f.g))

% generating another G and M
G = invdyn_lag_mdh_G_sym(q_r_sym, sym(mdh_matrix),sym(x_base), sym(franka.par_dyn_f.mass_all),...
                        sym(franka.par_dyn_f.com_all), sym(franka.par_dyn_f.g));
M = invdyn_lag_mdh_M_sym(q_r_sym, sym(mdh_matrix), sym(x_base),sym(franka.par_dyn_f.mass_all),...
                        sym(franka.par_dyn_f.com_all),sym(franka.par_dyn_f.inertia_all));

% [Tau, M, C, G] = invdyn_lag_mdh_sym(q_r_sym,q_rd_sym,sym(zeros(franka.nj,1)), sym(mdh_matrix),sym(x_base), sym(franka.par_dyn_f.mass_all),...
%                         sym(franka.par_dyn_f.com_all), sym(franka.par_dyn_f.inertia_all), sym(franka.par_dyn_f.g)); %

func_name = strcat('output/invdyn_lag_mdh_sym_', franka.name);
matlabFunction(M,"File",strcat(func_name,'_M'),...
        "Vars", var_name);
% 
func_name = strcat('output/invdyn_lag_mdh_sym_', franka.name);
matlabFunction(Tau,"File",strcat(func_name,'_Tau'),...
        "Vars", var_name);
% func_name = strcat('output/invdyn_lag_mdh_sym_', franka.name);
% matlabFunction(C,"File",strcat(func_name,'_C'),...
%         "Vars", var_name);
return

%% generating symbolic functions of M and G using NE

[M,FTau_G] = franka.invdyn_ne_w_end_sym_MG(1,1);

return
%% random motion gif
visual_kin = 1;
gifFile = 'examples\Franka\panda_gif.gif';
save_gif = 1;
q = rand(7,1);
dt = 0.1;
qd = zeros(7,1);
h = figure(3);
set(0,'defaultfigurecolor','w') 
set(groot,'defaulttextinterpreter','none');
set(h,'units','normalized','outerposition',[0 0 1 1])
set(h, 'Units', 'centimeters')
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)]);%
set(h, 'Units', 'centimeters')
for i = 1:100
    qdd = 0.1*rand(7,1)-0.05;
    qd = qd + qdd * dt;
    q = q + qd;
    franka.update_finger(q);
    if visual_kin

        subplot(1,2,1)
        franka.plot_finger(plot_par)
        grid off
        axis equal
        xlim([-0.8 0.8])
        ylim([-1 1])
        zlim([-1 1.3])
        axis manual
        drawnow
        hold off
        subplot(1,2,2)
        panda.plot(q','jointdiam',1)
        grid off
        axis equal
        xlim([-0.8 0.8])
        ylim([-1 1])
        zlim([-1 1.3])
        axis manual
        drawnow
        hold off
    end
    if save_gif
        % save as gif
        frame = getframe(gcf);
        img   = frame2im(frame);
        [A,map] = rgb2ind(img,256);
        % Write to GIF
        if i == 1
            imwrite(A,map,gifFile,'gif','LoopCount',Inf,'DelayTime',0.05);
        else
            imwrite(A,map,gifFile,'gif','WriteMode','append','DelayTime',0.05);
        end
    end
end
