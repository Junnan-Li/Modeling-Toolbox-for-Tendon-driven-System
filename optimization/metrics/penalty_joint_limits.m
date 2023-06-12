%   Penaliyation of the joint limits;
% 
% 
% source:
% [1] T. F. Chan and R. V Dubey, "A Weighted Least-Norm Solution Based Scheme 
% for Avoiding Joint Limits for Redundant Joint Manipulators," IEEE Trans. Robot. Autom., vol. 1, no. 2, 1995.
% [2] N. Vahrenkamp and T. Asfour, “Representing the robot’s workspace through 
% constrained manipulability analysis,” Auton. Robots, vol. 38, no. 1, pp. 17–30, 2015, doi: 10.1007/s10514-014-9394-z.



function  penal_jl = penalty_joint_limits(q, q_limit)

% q: [nj,n]
nj = size(q,2);
assert(~isempty(find(size(q_limit) ~= [nj 2])),'error: penal_jl with wrong input dimension!')

q_limits_low = q_limit(:,1);
q_limits_high = q_limit(:,2);

% H = 1/4 * (q_limits_high-q_limits_low)^2 ./ ((q_limits_high - q).*(q-q_limits_low));

Hdot = 1/1*(q_limits_high-q_limits_low).^2 .* (2*q - q_limits_high-q_limits_low) ./ ...
                                        ((q_limits_high - q).^2 .* (q-q_limits_low).^2);


penal_jl = 1./(sqrt(1+abs(Hdot)));
end