function compile_functions(obj)


nq = obj.nja;
% complie kinematic functions
compile_kinematics(nq);
% complie dynamics functions
compile_dynamics(nq);


end