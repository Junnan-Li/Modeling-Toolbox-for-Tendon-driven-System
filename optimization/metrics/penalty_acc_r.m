%   Penaliyation of the acceleration radius;
% 







function  penal_acc_r = penalty_acc_r(acc_r_normalized, par)

penal_acc_r = acc_r_normalized .^ (1/par);


end