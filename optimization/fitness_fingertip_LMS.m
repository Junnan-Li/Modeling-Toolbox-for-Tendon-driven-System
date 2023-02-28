








function fval = fitness_fingertip_LMS(opt_var,opt_var_index,opt_ft_limits,w_J_end_red,opt_fitness_optinos)


nvar = length(opt_var);
C = [];
if opt_fitness_optinos.opt_routing
    for i = 1:nvar/2
        if opt_var(i) == 0
            opt_var(i) = 0.1;
        end
        C(opt_var_index(nvar/2+i,1),opt_var_index(nvar/2+i,2)) = sign(opt_var(i))*opt_var(nvar/2+i);
    end
else
    for i = 1:nvar
        C(opt_var_index(i,1),opt_var_index(i,2)) = opt_var(i);
    end
end

[LMS_r, LMS_x, origin_included] = metric_fingertip_force_LMS(C, w_J_end_red, opt_ft_limits);
if ~origin_included
    fval = 1000;
else
    fval = 1/LMS_r;
end
end