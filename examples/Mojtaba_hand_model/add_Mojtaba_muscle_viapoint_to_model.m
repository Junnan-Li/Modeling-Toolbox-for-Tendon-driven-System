% plot the muscle path and distinguish the origin, viapoints, and insertion


function new_viapoint = add_Mojtaba_muscle_viapoint_to_model(muscle_name,Seg_name_in_table, Link, w_T_Link )


data_muscle = readcell('Mojtaba_model_data_muscle_path.csv');

num_vp = 0;
new_viapoint = [];
for i = 4:size(data_muscle,1)
    if any(~ismissing(data_muscle{i,2})) && any(~ismissing(data_muscle{i,3}))
        if isnumeric(data_muscle{i,5}) && matches(data_muscle{i,4},Seg_name_in_table) && ...
                matches(data_muscle{i,2},muscle_name)

            VP_name = strcat(Seg_name_in_table,'_',data_muscle{i,2},'_',data_muscle{i,3},'_',num2str(i));
            w_p_link = w_T_Link(1:3,4);
            w_R_link = w_T_Link(1:3,1:3);
            w_p_link_vp = [data_muscle{i,5:7}]'*1e-3 - w_p_link;
            new_viapoint_i = Link.add_viapoint_link(VP_name, w_R_link'*w_p_link_vp);
            new_viapoint = [new_viapoint;new_viapoint_i];
            num_vp = num_vp + 1;
        end
    end
end
fprintf('[add_Mojtaba_muscle_viapoint_to_model]: add %d Viapoints to the Link!  \n', num_vp)
