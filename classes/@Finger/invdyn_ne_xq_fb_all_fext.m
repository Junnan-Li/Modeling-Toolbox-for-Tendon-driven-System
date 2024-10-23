% inverse dynamic of the finger with the endeffector interaction to the
% environment with respect to the world frame
% floating base version 
% X_base represented in Euler angle xyz
% 
% call invdyn_ne_xq_mdh_all_fext.m
% 
% Note: in this function, obj.w_p_base & obj.w_R_base are not passed to the
% calculation, need to give as input xq
% input:
%           xq: [6+obj.nj]
%           xqD: [6+obj.nj] 
%           xqDD: [6+obj.nj]
%           F_ext_ne_all_fext: [6,obj.nj+2] [f_base;f_ext_joint;f_ee]
%                               external wrench of each frame
% 
% output:
%           FTau: [6+obj.nj] 
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% TODO: need to adapt to the passive joint mode

function [FTau] = invdyn_ne_xq_fb_all_fext(obj, varargin)
%xq, xqD, xqDD, F_ext_ne_all_fext)

if nargin == 1
    xq = [obj.w_p_base;R2euler_XYZ(obj.w_R_base);obj.q];
    xqD = zeros(obj.nj+6,1);
    xqDD = zeros(obj.nj+6,1);
    F_ext_ne_all_fext = zeros(6,obj.nj+2);
elseif nargin == 2
    xq = varargin{1};
    xqD = zeros(obj.nj+6,1);
    xqDD = zeros(obj.nj+6,1);
    F_ext_ne_all_fext = zeros(6,obj.nj+2);
elseif nargin == 3
    xq = varargin{1};
    xqD = varargin{2};
    xqDD = zeros(obj.nj+6,1);
    F_ext_ne_all_fext = zeros(6,obj.nj+2);
elseif nargin == 4
    xq = varargin{1};
    xqD = varargin{2};
    xqDD = varargin{3};
    F_ext_ne_all_fext = zeros(6,obj.nj+2);
elseif nargin == 5
    xq = varargin{1};
    xqD = varargin{2};
    xqDD = varargin{3};
    F_ext_ne_all_fext = varargin{4};
elseif nargin > 5
    error('[invdyn_ne_xq_fb_all_fext] too much input!')
end


assert(length(xq)== obj.nj+6, '[invdyn_ne_xq_fb_all_fext] input dimension is incorrect!')
assert(length(xqD)== obj.nj+6, '[invdyn_ne_xq_fb_all_fext] input dimension is incorrect!')
assert(length(xqDD)== obj.nj+6, '[invdyn_ne_xq_fb_all_fext] input dimension is incorrect!')
assert(all(size(F_ext_ne_all_fext)== [6,obj.nj+2]), '[invdyn_ne_xq_fb_all_fext] input dimension is incorrect!')

% n_state = obj.nj + 6;
mdh_ne = mdh_struct_to_matrix(obj.mdh_ori, 1);
mdh_ne(1:obj.nj,3) = mdh_ne(1:obj.nj,3);
Mass = obj.par_dyn_f.mass_all;
% X_base = zeros(6,1);
% X_base(1:3) = obj.w_p_base;
% X_base(4:6) = R2euler_XYZ(obj.w_R_base);
% XD_base = zeros(6,1);
% XDD_base = zeros(6,1);

CoM_ne = obj.par_dyn_f.com_all;
I_ne = obj.par_dyn_f.inertia_all;
g = obj.par_dyn_f.g;


[FTau,~] = invdyn_ne_xq_mdh_all_fext(xq,xqD,xqDD,...
            mdh_ne,Mass,F_ext_ne_all_fext, CoM_ne, I_ne, g);

end