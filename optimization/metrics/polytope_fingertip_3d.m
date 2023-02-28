%   generate fingertip force polytope (3d) from tendon space using MPT3 toolbox 
%   input:
%       M_coupling:         [njxnt] coupling matrix 
%       J_end_3d:           [3xnj] Jacobian matrix containing only
%                           translational term
%       force_limits:       [ntx2]  tendon force limits [min max]
%    
%   output:
%       P_ft:               Polyhedron [3] of fingertip force 
%       P:                  Polyhedron [nt+3] including tendon force 

function  [P_ft, P] = polytope_fingertip_3d(M_coupling, J_end_3d, force_limits)

% nj = size(M_coupling,1);
nt = size(M_coupling,2);
C = M_coupling;

pinv_w_J_end_T = pinv(J_end_3d');
pinv_w_J_end_T_C = pinv_w_J_end_T * C;

% inequalities for constraints of tendon force
H_1 = [eye(nt), zeros(nt,3);-eye(nt), zeros(nt,3)];
B_1 = [force_limits(:,2); -force_limits(:,1)];

% equality of tendon force to joint torque
% tau = A*f_t
He_1 = [pinv_w_J_end_T_C -eye(3)];
Be_1 = zeros(3,1);

% mpt toolbox define a polytope
P = Polyhedron('A', H_1, 'b', B_1, 'Ae', He_1, 'be', Be_1);
% eliminate the tendon force from variable
P_ft = [zeros(3,nt), eye(3)]*P;



end