% plot the muscle path and distinguish the origin, viapoints, and insertion


function muscle_all = add_Mojtaba_muscle_to_model(Mojtaba_hand_model, Seg_linK_mapping, varargin)

data_muscle = readcell('Mojtaba_model_data_muscle_path.csv');

if nargin == 2
    selected_muscle = '';
elseif nargin == 3
    selected_muscle = varargin{1};
end

curent_muscle = '';

muscle_all = {};
num_muscle = 0;
num_viapoint = 0;
for i = 4:size(data_muscle,1)
    if any(~ismissing(data_muscle{i,2})) && any(~ismissing(data_muscle{i,3})) ...
            && isnumeric(data_muscle{i,5})

        if isempty(selected_muscle) || matches(data_muscle{i,2},selected_muscle)
            muscle_name_i = data_muscle{i,2};
            if ~matches(curent_muscle,muscle_name_i)
                curent_muscle = muscle_name_i;
                muscle_i = Muscles(curent_muscle);
                Mojtaba_hand_model.add_Muscle(muscle_i);
                muscle_all{end+1} = muscle_i;
                num_muscle = num_muscle + 1;
            end
            Seg_name = data_muscle{i,4};
            row = find(ismember([Seg_linK_mapping(:,1)],Seg_name));
            Link_i = Seg_linK_mapping{row,2};
            VP_name = strcat(Link_i.name,'_',data_muscle{i,2},'_',data_muscle{i,3},'_',num2str(i));
            w_T_Link = Link_i.get_w_T_Link_inhand;
            w_p_link = w_T_Link(1:3,4);
            w_R_link = w_T_Link(1:3,1:3);
            w_p_link_vp = [data_muscle{i,5:7}]'*1e-3 - w_p_link;
            new_viapoint_i = Link_i.add_viapoint_link(VP_name, w_R_link'*w_p_link_vp);
            Mojtaba_hand_model.update_list_viapoints;
            muscle_i.add_viapoints(new_viapoint_i);

            num_viapoint = num_viapoint + 1;
        end
    end
end

fprintf('[add_Mojtaba_muscle_to_model]: add %d Viapoints to the Link!  \n', num_viapoint)
fprintf('[add_Mojtaba_muscle_to_model]: add %d Muscles to the Link!  \n', num_muscle)

