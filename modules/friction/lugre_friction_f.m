function f = lugre_friction_f(sigma0, sigma1, sigma2, z, dz, v)
% Lugre dynamic friction model
% 
% 
% source: 
% [1] K. J. Astrom and C. Canudas-De-Wit, “Revisiting the LuGre Friction Model,” 
% IEEE Control Syst., vol. 28, no. 6, pp. 101–114, 2008, doi: 10.1109/MCS.2008.929425.

f = sigma0*z + sigma1*dz + sigma2*v;

end