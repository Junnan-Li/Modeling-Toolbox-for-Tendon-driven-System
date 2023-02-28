%   metric of largest minimal sphere at joint torque level;



function  [LMS_r, LMS_x, origin_included] = metric_torque_LMS(M_coupling, force_limits)

nj = size(M_coupling,1);
% nt = size(M_coupling,2);

P_tau = polytope_torque(M_coupling, force_limits);

% origin point of the joint torque space
origin = zeros(nj,1);

% metrics
origin_included = P_tau.contains(origin);
if ~origin_included
    LMS_r = 0;
    LMS_x = 0;
else
    [LMS_r, LMS_x] = largest_minimum_radius_P_input(P_tau, origin);
end

end