% ststic friction model including stiction friction, stribeck friction and
% viscous friction
% input:
%       v: relative volecity
%       f_e: external force
%       parameter_struct: 
%           F_cp/F_cn:  positive/negative coulumb coefficient
%           F_sp/F_sn:  positive/negative stribeck coefficient
%           F_vp/F_vn:  positive/negative viscous coefficient
%           V_sp/V_sn:  positive/negative stribeck velocity
% 

% source:
% [1] F. Lange, M. Pfanne, F. Steinmetz, S. Wolf, and F. Stulp, 
% “Friction Estimation for Tendon-Driven Robotic Hands,” 
% Proc. - IEEE Int. Conf. Robot. Autom., vol. 2021-May, no. Icra, pp. 6505–6511, 2021.

function f = static_friction_f(v,f_n,f_s, parameter_struct)

if nargin == 1
    parameter_struct = static_friction_par_init();
    
end


F_cp = parameter_struct.F_cp;
F_cn = parameter_struct.F_cn;
F_sp = parameter_struct.F_sp;
F_sn = parameter_struct.F_sn;
F_vp = parameter_struct.F_vp;
F_vn = parameter_struct.F_vn;
V_sp = parameter_struct.V_sp;
V_sn = parameter_struct.V_np;

if v > 0
    f = f_n*(F_cp + (F_sp-F_cp)*exp(-v/V_sp) + F_vp*v);
elseif v < 0
    f = f_n*(F_cn + (F_sn-F_cn)*exp(-v/V_sn) + F_vn*v);
elseif f_s > f_n*F_sp
    f = f_n*F_sp;
elseif f_s < f_n*F_sn
    f = f_n*F_sn;
else
    f = f_s;
end

end


