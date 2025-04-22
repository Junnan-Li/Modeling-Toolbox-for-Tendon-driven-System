% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% Fixed base
% 
% input:
%           q:          [obj.nj]
%           varargin:
%               qD: 
%               qDD: 
%               F_ext_ne: [6x1]
%               mex:      [1] if use compiled mex function to compute
% 
% output:
%           Tau: [obj.nj] 
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% TODO: need to adapt to the passive joint mode

function Tau = invdyn_ne_w_end_T(obj, varargin)

if nargin == 1
    q = obj.q;
    qD = zeros(obj.nj,1);
    qDD = zeros(obj.nj,1);
    F_ext = zeros(6,1);
    mex = 0;
elseif nargin == 2
    q = varargin{1};
    qD = zeros(obj.nj,1);
    qDD = zeros(obj.nj,1);
    F_ext = zeros(6,1);
    mex = 0;
elseif nargin == 3
    q = varargin{1};
    qD = varargin{2};
    qDD = zeros(obj.nj,1);
    F_ext = zeros(6,1);
    mex = 0;
elseif nargin == 4
    q = varargin{1};
    qD = varargin{2};
    qDD = varargin{3};
    F_ext = zeros(6,1);
    mex = 0;
elseif nargin == 5
    q = varargin{1};
    qD = varargin{2};
    qDD = varargin{3};
    F_ext = varargin{4};
    mex = 0;
elseif nargin == 6
    q = varargin{1};
    qD = varargin{2};
    qDD = varargin{3};
    F_ext = varargin{4};
    mex = varargin{5};
elseif nargin > 6
    error('[invdyn_ne_w_end] too much input!')
end

assert(length(q)== obj.nj, '[invdyn_ne_w_end] dimension of joint vector is incorrect!')
assert(length(qD)== obj.nj, '[invdyn_ne_w_end] dimension of joint vector is incorrect!')
assert(length(qDD)== obj.nj, '[invdyn_ne_w_end] dimension of joint vector is incorrect!')
assert(length(F_ext)== 6, '[invdyn_ne_w_end] dimension of joint vector is incorrect!')
assert(length(qDD)== obj.nj, '[invdyn_ne_w_end] dimension of joint vector is incorrect!')
% if nargin == 5
%     mex = 0;
% elseif nargin == 6
%     mex = varargin{1};
% end

% mdh_ne = mdh_struct_to_matrix(obj.mdh_ori, 1);
% mdh_ne(1:obj.nj,3) = mdh_ne(1:obj.nj,3);
Mass = obj.par_dyn_f.mass_all;
% X_base = zeros(6,1);
% X_base(1:3) = obj.w_p_base;
% X_base(4:6) = R2euler_XYZ(obj.w_R_base);
XD_base = zeros(6,1);
XDD_base = zeros(6,1);
CoM = obj.par_dyn_f.com_all;
I = obj.par_dyn_f.inertia_all;
g = obj.par_dyn_f.g;

% if mex
%     [Tau,~,~] = invdyn_ne_mdh_mex(q,qD,qDD,mdh_ne, Mass,...
%         X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g);
% else
%     [Tau,~,~] = invdyn_ne_mdh(q,qD,qDD,mdh_ne, Mass,...
%         X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g);
% end
T = obj.get_T_all_links;
n_links = [0,1];
q_index = [1,obj.nj];
[Tau,~] = invdyn_ne_T(T,qD,qDD, n_links, q_index, Mass,...
    XD_base,XDD_base, F_ext, CoM, I, g);

end

