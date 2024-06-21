% IK using robotic system toolbox for Finger object
% 
% input:
%           x_des_all: desired position in world frame of al fingertips
%           iter_max: maximal number of iteration
%           tol: tolerance of the phi_x
%           alpha: change rate
% 
% output:
%           q: calculated joint angle
%           x_res: result of the position x
%           phi_x: PHI
%           iter: interation number 
% 
% Junnan Li, junnan.li@tum.de, 04.2024

function [q_res, solnInfo] = invkin_rst(obj,x_des)

assert(all(size(x_des) == [6,1]), 'Finger.invkin_rst: wrong dimension of X_des_all')


ik = inverseKinematics("RigidBodyTree",obj.update_rst_model);
% weights = [0 0 0 1 1 1];
% initialguess = homeConfiguration(puma);
endeff_name = obj.update_rst_model.BodyNames{end}();
w_T_ee_des = pR2T(x_des(1:3),euler2R_XYZ(x_des(4:6)));
[q_res,solnInfo] = ik(endeff_name,w_T_ee_des,ones(6,1),obj.q);

end
