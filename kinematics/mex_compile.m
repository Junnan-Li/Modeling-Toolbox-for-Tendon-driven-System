% compile kinematic function to mex with predefined input dimension








% code generation for n = 4
% ik_trans_numeric_joint_limits(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha)
% codegen ik_trans_numeric_joint_limits.m -args {zeros(5,4),zeros(3,1),zeros(4,1),zeros(4,2),0,0,0}


%% n = 4
% ik_trans_numeric_joint_limits_nullspace(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha,q_diff_min,mex)
codegen ik_trans_numeric_joint_limits_nullspace.m -args {zeros(5,4),zeros(3,1),zeros(4,1),zeros(4,2),0,0,0,0,1}




%% code generation for n = 5
% ik_trans_numeric_joint_limits(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha)
n = 5;
codegen ik_trans_numeric_joint_limits.m -args {zeros(n+1,4),zeros(3,1),zeros(n,1),zeros(n,2),0,0,0}


%% n = 5
% ik_trans_numeric_joint_limits_nullspace(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha,q_diff_min,mex)
codegen ik_trans_numeric_joint_limits_nullspace.m -args {zeros(n+1,4),zeros(3,1),zeros(n,1),zeros(n,2),0,0,0,0,1}








