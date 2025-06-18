% Forward dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% 
% input:
%           qD: 
%           Tau: 
%           F_ext: [6xn_q+2]
% 
% output:
%           qDD: [obj.nj] 
% 
% 
% TODO: need to adapt to the passive joint mode

function [qDD,M_fd,C_fd,G_fd] = fordyn_ne_w_end(obj, q, qD, Tau, F_ext, varargin)

assert(length(q)== obj.nj, 'dimension of joint vector is incorrect!')
assert(length(qD)== obj.nj, 'dimension of joint vector is incorrect!')
assert(length(Tau)== obj.nj, 'dimension of joint vector is incorrect!')

if nargin == 5
    mex = 0;
elseif nargin == 6
    mex = varargin{1};
end

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

if obj.kin_use_T
    T = obj.get_T_all_links;
    n_links = [0,1];
    q_index = [1,obj.nj];
    [qDD,M_fd,C_fd,G_fd] = fordyn_ne_T(T,qD,Tau, n_links, q_index, Mass, ...
        XD_base,XDD_base, F_ext_ne, CoM_ne, I_ne, g);
else

    mdh_ne = mdh_struct_to_matrix(obj.mdh_ori, 1); % use mdh_ori without counting q
    mdh_ne(1:obj.nj,3) = mdh_ne(1:obj.nj,3);

    [qDD,M_fd,C_fd,G_fd] = fordyn_ne_mdh(q,qD,Tau,mdh_ne, Mass,...
        X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g, mex);
end
end