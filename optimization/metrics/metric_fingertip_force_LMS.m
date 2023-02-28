%   metric of largest minimal sphere at joint torque level;



function  [LMS_r, LMS_x, origin_included] = metric_fingertip_force_LMS(M_coupling, w_J_end_red, force_limits)

% nj = size(M_coupling,1);
% nt = size(M_coupling,2);

P_ft = polytope_fingertip_3d(M_coupling, w_J_end_red, force_limits);

% origin point of the joint torque space
origin = zeros(3,1);
origin_included = P_ft.contains(origin);

if ~origin_included
    LMS_r = 0;
    LMS_x = 0;
else
    [LMS_r, LMS_x] = largest_minimum_radius_P_input(P_ft, origin);
end

end