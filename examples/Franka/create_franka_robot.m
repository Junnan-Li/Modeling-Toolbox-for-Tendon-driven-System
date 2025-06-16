
franka = create_finger_random('Franka', 7);

mdh_default_struct = franka.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);
mdh_matrix = [0,     0,   0,0.333;...
              -pi/2, 0,   0, 0;...
              pi/2,  0,   0, 0.316;...
              pi/2,  0.0825, 0, 0; ...
              -pi/2, -0.0825, 0, 0.384;...
              pi/2,  0,   0, 0;...
              pi/2,  0.088, 0, 0.107;
              0,     0,     0, 0];


franka.set_mdh_parameters(mdh_matrix);
franka.set_base([0,0,0]',eye(3))
q = zeros(franka.nj,1);
franka.update_finger(q);


plot_par = franka.plot_parameter_init();


figure(1)
franka.plot_finger(plot_par)

grid on
axis equal
