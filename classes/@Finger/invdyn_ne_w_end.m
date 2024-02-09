% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% 
% input:
%           qD: 
%           qDD: 
%           F_ext_ne: [6x1]
% 
% output:
%           Tau: [obj.nj] 
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% TODO: need to adapt to the passive joint mode

function Tau = invdyn_ne_w_end(obj, q, qD, qDD, F_ext, varargin)

assert(length(q)== obj.nj, 'dimension of joint vector is incorrect!')
assert(length(qD)== obj.nj, 'dimension of joint vector is incorrect!')
assert(length(qDD)== obj.nj, 'dimension of joint vector is incorrect!')

if nargin == 5
    mex = 0;
elseif nargin == 6
    mex = varargin{1};
end

mdh_ne = mdh_struct_to_matrix(obj.mdh_ori, 1);
mdh_ne(1:obj.nj,3) = mdh_ne(1:obj.nj,3);
Mass = obj.par_dyn_f.mass_all;
X_base = zeros(6,1);
X_base(1:3) = obj.w_p_base;
X_base(4:6) = R2euler_XYZ(obj.w_R_base);
XD_base = zeros(6,1);
XDD_base = zeros(6,1);
F_ext_ne = F_ext;
CoM_ne = obj.par_dyn_f.com_all;
I_ne = obj.par_dyn_f.inertia_all;
g = obj.par_dyn_f.g;

if mex
    [Tau,~,~] = invdyn_ne_mdh_mex(q,qD,qDD,mdh_ne, Mass,...
        X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g);
else
    [Tau,~,~] = invdyn_ne_mdh(q,qD,qDD,mdh_ne, Mass,...
        X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g);
end

end