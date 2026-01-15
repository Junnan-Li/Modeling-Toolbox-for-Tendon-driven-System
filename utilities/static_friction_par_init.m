% initialize the parameter_struct for the function static_friction_f.m
% output:
%       v: relative volecity
%       parameter_struct: 
%           F_cp/F_cn:  positive/negative coulumb coefficient
%           F_sp/F_sn:  positive/negative stribeck coefficient
%           F_vp/F_vn:  positive/negative viscous coefficient
%           V_sp/V_sn:  positive/negative stribeck velocity
function parameter_struct = static_friction_par_init()

parameter_struct = struct();

parameter_struct.F_cp = 0.3;
parameter_struct.F_cn = -0.3;
parameter_struct.F_sp = 0.5;
parameter_struct.F_sn = -0.5;
parameter_struct.F_vp = 0.02;
parameter_struct.F_vn = 0.02;
parameter_struct.V_sp = 0.9;
parameter_struct.V_np = -0.9;
end