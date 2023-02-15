% using characteristic of pseudo-inverse (minimum 2-norm solution of Ax=b)
% to realize the realtime calculation of nullspace  
% 
% "Tendon Impedance Control of the Human Finger" Tantau, Mathias 2017



function [f_tendon, tau_real] = tda_s1(tau_d,Coupling_matrix,min_force,max_force, num_iteration,tol)
%#codegen


tau_tol = tol;
pinv_C = pinv(Coupling_matrix);
% N_C = null(Coupling_matrix);
num_tendon = size(Coupling_matrix,2);

f_tendon = min_force;
% tic
for i = 1:num_iteration
    
    tau_rem = tau_d - Coupling_matrix * f_tendon;
    if max(abs(tau_rem)) < tau_tol
        break
    end   
    f_tendon = f_tendon + pinv_C*tau_rem ;
    f_tendon(f_tendon(:)<min_force ) = min_force(f_tendon(:)<min_force );
    f_tendon(f_tendon(:)>max_force ) = max_force(f_tendon(:)>max_force );   
end
% timecost = toc;
tau_real = Coupling_matrix * f_tendon;

end

