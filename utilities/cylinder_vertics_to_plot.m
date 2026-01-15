% calculate cylinder samples points for plotting
% with given radius, height, center position, orientation
% 

function [X,Y,Z] = cylinder_vertics_to_plot(w_T_cyl, r,h,n)

assert(all(size(w_T_cyl)== [4,4]), '[cylinder_vertics_to_plot]: dimension of p_center is incorrect!')


[X_ori,Y_ori,Z_ori] = cylinder(r,n);
length = h;
Z_ori(1,:) = -length/2;
Z_ori(2,:) = length/2;
p_1 = [X_ori(1,:);Y_ori(1,:);Z_ori(1,:);ones(1,n+1)];
p_2 = [X_ori(2,:);Y_ori(2,:);Z_ori(2,:);ones(1,n+1)];

p_1_oriented = w_T_cyl * p_1;
p_2_oriented = w_T_cyl * p_2;

X = [p_1_oriented(1,:);p_2_oriented(1,:)];
Y = [p_1_oriented(2,:);p_2_oriented(2,:)];
Z = [p_1_oriented(3,:);p_2_oriented(3,:)];
end