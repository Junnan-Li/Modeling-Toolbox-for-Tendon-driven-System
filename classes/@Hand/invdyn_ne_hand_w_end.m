% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% 
% input:
%           q:  [obj.nj] 
%           varargin
%               qD: [obj.nj] 
%               qDD: [obj.nj] 
%               F_ext_ne: [6,obj.nf] external wrench at each fingertips 
%               mex: [1] if use mex compiled function
% 
% output:
%           Tau: [obj.nj] 
% 
% 
% TODO: need to adapt to the passive joint mode

function Tau = invdyn_ne_hand_w_end(obj, q, varargin)

if nargin == 2
    qD = zeros(obj.nj,1);
    qDD = zeros(obj.nj,1);
    F_ext = zeros(6,obj.nf);
    mex = 0;
elseif nargin == 3
    qD = varargin{1};
    qDD = zeros(obj.nj,1);
    F_ext = zeros(6,obj.nf);
    mex = 0;
elseif nargin == 4
    qD = varargin{1};
    qDD = varargin{2};
    F_ext = zeros(6,obj.nf);
    mex = 0;
elseif nargin == 5
    qD = varargin{1};
    qDD = varargin{2};
    F_ext = varargin{3};
    mex = 0;
elseif nargin == 6
    qD = varargin{1};
    qDD = varargin{2};
    F_ext = varargin{3};
    mex = varargin{4};
else
    error('[invdyn_ne_hand_w_end] input is incorrect!')
end
assert(length(q)== obj.nj, '[invdyn_ne_hand_w_end] dimension of q is incorrect!')
assert(length(qD)== obj.nj, '[invdyn_ne_hand_w_end] dimension of qd is incorrect!')
assert(length(qDD)== obj.nj, '[invdyn_ne_hand_w_end] dimension of qdd is incorrect!')

T = obj.get_w_T_all();
Mass = [];
CoM = [];
I = [];

% kinematic description
kin_str = struct();
kin_str.nb = obj.nb;
kin_str.njb = zeros(obj.nb,1);
kin_str.nf = obj.nf;
kin_str.njf = zeros(obj.nf,1);

if obj.nb ~= 0
    for i = 1:obj.nb
        base_i = obj.base(i);
        Mass = [Mass;obj.par_dyn_h.mass_all{i,1}];
        CoM = [CoM,obj.par_dyn_h.com_all{i,1}];
        I = [I,obj.par_dyn_h.inertia_all{i,1}];
        kin_str.njb(i) = base_i.nj;
    end
end
if obj.njf ~= 0
    for i = 1:obj.nf
        finger_i = obj.list_fingers(i);
        Mass = [Mass;obj.par_dyn_h.mass_all{i,2}];
        CoM = [CoM,obj.par_dyn_h.com_all{i,2}];
        I = [I,obj.par_dyn_h.inertia_all{i,2}];
        kin_str.njf(i) = finger_i.nj;
    end
end

X_base = zeros(6,1);
X_base(1:3) = obj.w_p_base;
X_base(4:6) = R2euler_XYZ(obj.w_R_base);
XD_base = zeros(6,1);
XDD_base = zeros(6,1);
g = obj.par_dyn_h.g;


if mex
%     [Tau,~,~] = invdyn_ne_mdh_mex(q,qD,qDD,mdh_ne, Mass,...
%         X_base, XD_base, XDD_base, F_ext_ne, CoM_ne, I_ne, g);
    error('[invdyn_ne_hand_w_end] not implemented yet!')
else
    [Tau,F] = invdyn_ne_T(q,qD,qDD,T, kin_str, Mass, ...
        X_base, XD_base,XDD_base, F_ext, CoM, I, g);
end

end