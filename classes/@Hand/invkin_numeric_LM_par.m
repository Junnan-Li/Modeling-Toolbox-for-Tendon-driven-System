% IK parameter template for IK_numerix_LM function 
% 
% Junnan Li, junnan.li@tum.de, 06.2024

function par_Struct = invkin_numeric_LM_par(obj)

par_Struct = struct();
par_Struct.iter_max = 100;
par_Struct.retry_num = 10;
par_Struct.tol = [1e-6*ones(3,1);1e-5*ones(3,1)];
par_Struct.W_e = diag([1,1,1,1,1,1]);
par_Struct.W_d = 1e-6*diag(ones(obj.nj,1));
par_Struct.visual = 0;

end