% create Mojtaba thumb from the csv file 


function finger = create_Mojtaba_thumb_from_csv(finger_name)


data_cylinder = readcell('Mojtaba_model_data_cylinder.csv');
data_landmarker = readcell('Mojtaba_model_data_landmarker.csv');
data_axes = readcell('Mojtaba_model_data_axes.csv');


cylinder_position = [];
landmarker_position = [];
cylinder_axis = [];
for i = 3:size(data_cylinder,1)
    if contains(data_cylinder{i,1},num2str(1)) ...
            && isnumeric(data_cylinder{i,3})
        cylinder_position = [cylinder_position; ...
        [data_cylinder{i,5},data_cylinder{i,6},data_cylinder{i,7}]*0.001];
    end
end

for i = 2:size(data_axes,1)
    if contains(data_axes{i,1},num2str(1)) && contains(data_axes{i,2},{'Flex','Abd'}) ...
            && isnumeric(data_axes{i,3})
        cylinder_axis = [cylinder_axis; ...
        [data_axes{i,3},data_axes{i,4},data_axes{i,5}]];
    end
end


for i = 2:size(data_landmarker,1)
    if contains(data_landmarker{i,1},num2str(1)) && contains(data_landmarker{i,1},{'h','b'}) ...
            && isnumeric(data_landmarker{i,3})
        landmarker_position = [landmarker_position; ...
        [data_landmarker{i,3},data_landmarker{i,4},data_landmarker{i,5}]*0.001];
    end
end
length_MP = norm(landmarker_position(2,:)'- landmarker_position(1,:)',2);
length_PP = norm(landmarker_position(3,:)'- landmarker_position(2,:)',2);
length_DP =  norm(landmarker_position(4,:)'- landmarker_position(3,:)',2);

finger_dimension_3 = [0,length_MP,length_PP,length_DP]; % in meter
finger  = Finger(finger_name, 'type','R_RRRR', 'l_links',finger_dimension_3);
mdh_default_struct = finger.mdh_ori;
mdh_matrix = mdh_struct_to_matrix(mdh_default_struct, 1);

theta1 = -acos(([1,0]*(landmarker_position(2,1:2)'- landmarker_position(1,1:2)'))/norm(landmarker_position(2,1:2)'- landmarker_position(1,1:2)'));
R_base = euler2R_XYZ([0,0,theta1]);
cylinder_axis_rotated = (R_base * cylinder_axis')';
mdh_matrix(1,1) = -acos(([0,1] * cylinder_axis_rotated(1,[2,3])')/norm( cylinder_axis_rotated(1,[2,3])')) ; % MCP_Flex
mdh_matrix(2,1) = -mdh_matrix(1,1) + acos([0,1] * cylinder_axis_rotated(2,[2,3])'/norm(cylinder_axis_rotated(2,[2,3])')) - pi;% MCP_abd
mdh_matrix(3,1) = - mdh_matrix(1,1) - mdh_matrix(2,1) + acos([0,1] * cylinder_axis_rotated(3,[2,3])'/norm( cylinder_axis_rotated(3,[2,3])')) - pi;% PIP
mdh_matrix(4,1) = - mdh_matrix(1,1) - mdh_matrix(2,1) - mdh_matrix(3,1) ...
                + acos([0,1] * cylinder_axis_rotated(4,[2,3])'/norm( cylinder_axis_rotated(4,[2,3])')) - pi;% PIP
finger.set_mdh_parameters(mdh_matrix);

% set base information
% p_32 = cylinder_position(2,1:2)'- cylinder_position(1,1:2)';
% eul_base = -acos(([1,0]*p_32)/norm(p_32));
finger.set_base(landmarker_position(1,:)',R_base);
finger.update_finger(zeros(finger.nj,1));

end