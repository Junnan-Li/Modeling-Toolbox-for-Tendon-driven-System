





function fval = fitness_metric_three_fingers_grasp(X_object,r_object,Metric)


p_object = X_object(1:3);
phi_object = X_object(4:6);
p_c_thumb = r_object * euler2R_XYZ(phi_object)*euler2R_XYZ([-pi/2,0,0])*[0,1,0]' + p_object';
p_c_index = r_object * euler2R_XYZ(phi_object)*euler2R_XYZ([pi/3,0,0])*[0,1,0]' + p_object';
p_c_middle = r_object * euler2R_XYZ(phi_object)*euler2R_XYZ([pi*2/3,0,0])*[0,1,0]' + p_object';


[k_thumb,dist_thumb] = dsearchn(Metric.thumb.pos,p_c_thumb');
[k_index,dist_index] = dsearchn(Metric.fingers.pos,p_c_index');
[k_middle,dist_middle] = dsearchn(Metric.middle.pos,p_c_middle');
if dist_thumb >= 5e-3
    metric_thumb = 0;
else
    metric_thumb = max(Metric.thumb.metric_n(k_thumb,:));
end
if dist_index >= 5e-3
    metric_index = 0;
else
    metric_index = max(Metric.fingers.metric_n(k_index,:));
end
if dist_middle >= 5e-3
    metric_middle = 0;
else
    metric_middle = max(Metric.middle.metric_n(k_middle,:));
end
fval = -metric_thumb*metric_index*metric_middle;
end