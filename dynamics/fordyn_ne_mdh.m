% recursive Newton-Euler method to compute inverse dynamic with mdh
% parameters
% Input:
%     q,qD:             [n_qx1] joint states
%     Tau               [n_q] joint torque
%     mdh:              [n_q+1x4] in sequence of [alpha,a,theta,d] To be
%                       noted that the theta should exclude from mdh parameters 
%     Mass:             [n_q+1] mass vector. the first mass is the base 
%     X_base:           [6x1] eulerxyz
%     XD_base:          [6x1] translational and angular velocity
%     F_ext:            [6x1] force/moment that exerted by the environment to the endeffector frame. 
%     CoM:              [3xn_q+1] in local frame; last column is endeffector
%     I:                [6xn_q+1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
%     g:                [3x1] gravity term
% 
% Output:
%     qDD:              [n_qx1] joint acceleration 
%     M_fd:             [n_qxn_q] Mass matrix 
%     C_fd:             [n_q] velocity-dependent torque 
%     G_fd:             [n_q] gravity torque  

function [qDD,M_fd,C_fd,G_fd] = fordyn_ne_mdh(q,qD,Tau,mdh, Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g, varargin)

n_q = length(q);
assert(n_q == length(qD)&&n_q == length(Tau),'fordyn_ne_mdh: dimension error!');

if nargin == 12
    mex = 1;
elseif nargin == 13
    mex = varargin{1};
end

M_fd = zeros(n_q,n_q);
G_fd = zeros(n_q,1);
C_fd = zeros(n_q,1);

% gravity term
if mex
    G_fd = invdyn_ne_mdh_mex(q,zeros(n_q,1),zeros(n_q,1),mdh, Mass,...
        X_base, XD_base, XDD_base, zeros(6,1), CoM, I, g);
else
    G_fd = invdyn_ne_mdh(q,zeros(n_q,1),zeros(n_q,1),mdh, Mass,...
        X_base, XD_base, XDD_base, zeros(6,1), CoM, I, g);
end
% C term
if mex
    C_fd = invdyn_ne_mdh_mex(q,qD,zeros(n_q,1),mdh, Mass,...
        X_base, XD_base, XDD_base, zeros(6,1), CoM, I, g);
else
    C_fd = invdyn_ne_mdh(q,qD,zeros(n_q,1),mdh, Mass,...
             X_base, XD_base, XDD_base, zeros(6,1), CoM, I, g);
end
C_fd = C_fd - G_fd;

% M term
M_fd = zeros(n_q,n_q);

for i = 1:n_q
    qDD_M = zeros(n_q,1);
    qDD_M(i) = 1;
    if mex
        M_fd(:,i) = invdyn_ne_mdh_mex(q,zeros(n_q,1),qDD_M,mdh, Mass,...
            X_base, XD_base, XDD_base, zeros(6,1), CoM, I, g);
    else
        M_fd(:,i) = invdyn_ne_mdh(q,zeros(n_q,1),qDD_M,mdh, Mass,...
            X_base, XD_base, XDD_base, zeros(6,1), CoM, I, g);
    end
    M_fd(:,i) = M_fd(:,i) - G_fd;
end

% use Jacobian_geom_mdh.m to calculate geometric Jacobian
w_R_b = euler2R_XYZ(X_base(4:6));
b_J = Jacobian_geom_mdh(mdh,q);
J_ne = blkdiag(w_R_b,w_R_b)*b_J;

qDD = (M_fd)\(Tau + J_ne'*F_ext - G_fd - C_fd);


end