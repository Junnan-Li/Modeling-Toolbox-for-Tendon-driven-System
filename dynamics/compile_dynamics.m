function compile_dynamics(nq)



% ik_trans_numeric_joint_limits(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha)
codegen invdyn_ne_mdh.m -args {zeros(nq,1),zeros(nq,1),zeros(nq,1),zeros(nq+1,4),zeros(nq+1,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(3,nq+1),zeros(6,nq+1),zeros(3,1)}
movefile ./invdyn_ne_mdh_mex.mexw64 ./dynamics/invdyn_ne_mdh_mex.mexw64


% ik_trans_numeric_joint_limits_nullspace(mdh_matrix,x_des,q_init,q_limits,iter_max,tol,alpha,q_diff_min,mex)
codegen invdyn_ne_mdh_all_fext.m -args {zeros(nq,1),zeros(nq,1),zeros(nq,1),zeros(nq+1,4),zeros(nq+1,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,nq+2),zeros(3,nq+1),zeros(6,nq+1),zeros(3,1)}
movefile ./invdyn_ne_mdh_all_fext_mex.mexw64 ./dynamics/invdyn_ne_mdh_all_fext_mex.mexw64
end