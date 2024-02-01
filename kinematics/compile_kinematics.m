
function compile_kinematics(nq)



% ik_trans_numeric_joint_limits(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha)
codegen ik_trans_numeric_joint_limits.m -args {zeros(nq+1,4),zeros(3,1),zeros(nq,1),zeros(nq,2),0,0,0};
movefile ./ik_trans_numeric_joint_limits_mex.mexw64 ./output/ik_trans_numeric_joint_limits_mex.mexw64

% ik_trans_numeric_joint_limits_nullspace(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha,q_diff_min,mex)
codegen ik_trans_numeric_joint_limits_nullspace.m -args {zeros(nq+1,4),zeros(3,1),zeros(nq,1),zeros(nq,2),0,0,0,0,1};
movefile ./ik_trans_numeric_joint_limits_nullspace_mex.mexw64 ./output/ik_trans_numeric_joint_limits_nullspace_mex.mexw64
end