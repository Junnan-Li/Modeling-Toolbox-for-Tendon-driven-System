% recursive Newton-Euler method to compute inverse dynamic with
% transformation matrix
% Input:
%     q,qD:             [n_q] joint states
%     Tau               [n_q] joint torque
%     T:                [4x4x n_frame] each Finger object has nqi+2  
%     kin_str           [struct] nb: number of bases; njb [nb,1]: nq of
%
%     X_base:           [6x1] eulerxyz
%     XD_base:          [6x1] translational and angular velocity
%     F_ext:            [6xn_q+2] force/moment that exerted by the environment to the endeffector frame. 
%     CoM:              [3xn_q+1] in local frame; last column is endeffector
%     I:                [6xn_q+1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
%     g:                [3x1] gravity term in world frame
% 
% Output:
%     qDD:              [n_q] joint acceleration 
%     M_fd:             [n_qxn_q] Mass matrix 
%     C_fd:             [n_q] velocity-dependent torque 
%     G_fd:             [n_q] gravity torque  

function [qDD,M,C,G] = fordyn_ne_T(q,qD,Tau,T, kin_str, Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g, varargin)

n_q = length(q);
assert(n_q == length(qD),'[invdyn_ne_T] dimension error!');
n_b = kin_str.nb; % number of bases
n_f = kin_str.nf; % number of fingers
njb = kin_str.njb; % number of joints of each base
njf = kin_str.njf; % number of joints of each finger
n_frame = n_q + 2*(n_b+n_f); % each Finger has nqi + 2 frames (except world frame) 
assert(n_b == length(njb) && n_f == length(njf),'[fordyn_ne_T] kin_str input error!');
assert(n_q == sum(njb) + sum(njf),'[fordyn_ne_T] kin_str input error!');

assert(all(size(F_ext) == [6,n_f]),'[fordyn_ne_T] F_ext dimension error!');
assert(all(size(T) == [4,4,n_frame]),'[fordyn_ne_T] T dimension error!');

if nargin == 13
    mex = 0;
elseif nargin == 14
    mex = varargin{1};
end

M = zeros(n_q,n_q);
G = zeros(n_q,1);
C = zeros(n_q,1);

% gravity term
if mex
    G = invdyn_ne_T_mex(q,zeros(n_q,1),zeros(n_q,1),T, kin_str, ...
        Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g);
else
    G = invdyn_ne_T(q,zeros(n_q,1),zeros(n_q,1),T, kin_str, ...
        Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g);
end
% C term
if mex
    C = invdyn_ne_T_mex(q,qD,zeros(n_q,1),T, kin_str, ...
        Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g);
else
    C = invdyn_ne_T(q,qD,zeros(n_q,1),T, kin_str, ...
        Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g);
end
C = C - G;

% M term
for i = 1:n_q
    qDD_M = zeros(n_q,1);
    qDD_M(i) = 1;
    if mex
        M(:,i) = invdyn_ne_T_mex(q,zeros(n_q,1),qDD_M,T, kin_str, ...
        Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g);
    else
        M(:,i) = invdyn_ne_T(q,zeros(n_q,1),qDD_M,T, kin_str, ...
        Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g);
    end
    M(:,i) = M(:,i) - G;
end

% use Jacobian_geom_mdh.m to calculate geometric Jacobian
% w_R_b = euler2R_XYZ(X_base(4:6));
% b_J = Jacobian_geom_mdh(mdh,q);
% J_ne = blkdiag(w_R_b,w_R_b)*b_J;
tau_ext = zeros(n_q,1);
% for i = 1:n_q+1
%     mdh_i = mdh(1:i,:);
%     q_i = q(1:min(i,n_q));
%     b_J_i = zeros(6,n_q);
%     b_J_i(:,1:min(i,n_q)) =  Jacobian_geom_mdh(mdh_i,q_i);
%     J_ne_i= blkdiag(w_R_b,w_R_b)*b_J_i;
%     tau_i = J_ne_i'*F_ext(:,i+1);
%     tau_ext = tau_ext + tau_i;
% end


qDD = (M)\(Tau + tau_ext - G - C);


end