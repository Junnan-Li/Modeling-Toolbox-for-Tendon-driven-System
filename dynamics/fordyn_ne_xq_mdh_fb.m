% recursive Newton-Euler method to compute inverse dynamic with mdh
% parameters
% Input:
%     xq:               [n_q+6x1] [X_base; q] state vector
%     xqD:              [n_q+6x1] [XD_base; qD] velocity state vector
%     FTau:             [n_q+6x1] [F_base; torque]
%     mdh:              [n_q+1x4] in sequence of [alpha,a,theta,d] To be
%                       noted that the theta should exclude from mdh parameters 
%     Mass:             [n_q+1] mass vector. the first mass is the base 
%     F_ext:            [6x1] force/moment that exerted by the environment to the endeffector frame. 
%     CoM:              [3xn_q+1] in local frame;
%     I:                [6xn_q+1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
%     g:                [3x1] gravity term
% 
% Output:
%     xqDD:             [n_q+6x1] [XD_base; qD] acceleration state vector
%     M_xq:             [n_q+6xn_q+6] Mass matrix 
%     C_xq:             [n_q+6] velocity-dependent torque 
%     G_xq:             [n_q+6] gravity torque  

function [xqDD,M_xq,C_xq,G_xq] = fordyn_ne_xq_mdh_fb(xq,xqD,FTau,mdh, Mass, F_ext, CoM, I, g)


X_base = xq(1:6);
q = xq(7:end);
n_state = length(xq);
n_q = length(q);
% assert(n_q == length(qD)&&n_q == length(Tau),'fordyn_ne_mdh: dimension error!');

% if nargin == 12
%     mex = 1;
% elseif nargin == 13
%     mex = varargin{1};
% end

M_xq = zeros(n_state,n_state);
G_xq = zeros(n_state,1);
C_xq = zeros(n_state,1);

%% gravity term
% xqD = 0; xqDD = 0; F_ext = 0;

[FTau_G,~] = invdyn_ne_xq_mdh_all_fext(xq,zeros(n_state,1),zeros(n_state,1),mdh,Mass,zeros(size(F_ext)), CoM, I, g);
G_xq = FTau_G;

%% C term
% xqDD = 0; F_ext = 0
[FTau_CG,~] = invdyn_ne_xq_mdh_all_fext(xq,xqD,zeros(n_state,1),mdh,Mass,zeros(size(F_ext)), CoM, I, g);
C_xq = FTau_CG - G_xq;

%% M term
% xqD = 0, F_ext = 0

for i = 1:n_state
    xqDD_i = zeros(n_state,1);
    xqDD_i(i) = 1;
    [FTau_CG,~] = invdyn_ne_xq_mdh_all_fext(xq,zeros(n_state,1),xqDD_i,mdh,Mass,zeros(size(F_ext)), CoM, I, g);
    M_xq(:,i) = FTau_CG - G_xq;
end

% use Jacobian_geom_mdh.m to calculate geometric Jacobian
w_R_b = euler2R_XYZ(X_base(4:6));
% b_J = Jacobian_geom_mdh(mdh,q);
% J_ne = blkdiag(w_R_b,w_R_b)*b_J;
tau_ext = zeros(n_q,1);
for i = 1:n_q+1
    mdh_i = mdh(1:i,:);
    q_i = q(1:min(i,n_q));
    b_J_i = zeros(6,n_q);
    b_J_i(:,1:min(i,n_q)) =  Jacobian_geom_mdh(mdh_i,q_i);
    J_ne_i= blkdiag(w_R_b,w_R_b)*b_J_i;
    tau_i = J_ne_i'*F_ext(:,i+1);
    tau_ext = tau_ext + tau_i;
end
Ftau_ext = [F_ext(:,1);tau_ext];

xqDD = (M_xq)\(FTau + Ftau_ext - G_xq - C_xq);


end