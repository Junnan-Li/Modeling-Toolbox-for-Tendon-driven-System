%   generate fingertip force polytope (3d) from tendon space using MPT3 toolbox 
%   input:
%       M_coupling:         [njxnt] coupling matrix 
%       force_limits:       [ntx2]  tendon force limits [min max]
%    
%   output:
%       P_tau:               Polyhedron [3] of torque 
%       P:                  Polyhedron [nt+3] including tendon force 

function  [P_tau, P] = polytope_torque(M_coupling, force_limits)

nj = size(M_coupling,1);
nt = size(M_coupling,2);
C = M_coupling;


% inequalities for constraints of torque
H_1 = [eye(nt), zeros(nt,nj);-eye(nt), zeros(nt,nj)];
B_1 = [force_limits(:,2); -force_limits(:,1)];

% equality of tendon force to joint torque
% tau = A*f_t
He_1 = [C -eye(nj)];
Be_1 = zeros(nj,1);

% mpt toolbox define a polytope
P = Polyhedron('A', H_1, 'b', B_1, 'Ae', He_1, 'be', Be_1);
% eliminate the tendon force from variable
P_tau = [zeros(nj,nt), eye(nj)]*P;



end