function dfdt = dahl_friction_dfdt(t,f,v,sigma,F_c)
% Dahl dynamic friction model
% 
% 
% source: 
% [1] K. J. Astrom and C. Canudas-De-Wit, “Revisiting the LuGre Friction Model,” 
% IEEE Control Syst., vol. 28, no. 6, pp. 101–114, 2008, doi: 10.1109/MCS.2008.929425.


dfdt = sigma*( v - 1/F_c*abs(v)*f );

end

