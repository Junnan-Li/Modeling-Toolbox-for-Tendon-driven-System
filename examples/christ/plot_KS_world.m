function plot_KS_world(TF, la, lw)
% TU Munich - Munich School of Robotics and Machine Intelligence
% Alexander Toedtheide 2019 (C)

TF_names = fieldnames(TF.KSW);
for i = 1:length(TF_names)
  
    if strcmp(TF_names{i}, 'KS_0_B1') || strcmp(TF_names{i}, 'KS_0_B2') || strcmp(TF_names{i}, 'KS_0_B3')
        T = TF.KSW.(TF_names{i})*trotz(pi);
    else
        T = TF.KSW.(TF_names{i});
    end
  sp = T(1:3,4);
  x_ee = (T(1:3,4)+la*T(1:3,1));
  y_ee = (T(1:3,4)+la*T(1:3,2));
  z_ee = (T(1:3,4)+la*T(1:3,3));

hold on;
plot3([sp(1) x_ee(1)],[sp(2) x_ee(2)], [sp(3) x_ee(3)], 'r', 'LineWidth', lw); hold on;
plot3([sp(1) y_ee(1)],[sp(2) y_ee(2)], [sp(3) y_ee(3)], 'g', 'LineWidth', lw); hold on;
plot3([sp(1) z_ee(1)],[sp(2) z_ee(2)], [sp(3) z_ee(3)], 'b', 'LineWidth', lw); hold on;
    
end

end
