function dzdt = lugre_friction_dzdt(t,z,v,sigma,F_c,F_s,v_s,alpha)
% Lugre dynamic friction model
% 
% 
% source: 
% [1] K. J. Astrom and C. Canudas-De-Wit, “Revisiting the LuGre Friction Model,” 
% IEEE Control Syst., vol. 28, no. 6, pp. 101–114, 2008, doi: 10.1109/MCS.2008.929425.

g_v = F_c + (F_s - F_c)*exp(-(abs(v/v_s))^alpha);

dzdt = v - sigma*abs(v)/g_v * z;

end