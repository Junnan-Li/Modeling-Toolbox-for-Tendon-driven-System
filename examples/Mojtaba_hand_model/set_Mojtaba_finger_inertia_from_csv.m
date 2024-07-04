% create Mojtaba finger from the csv file 


function finger = set_Mojtaba_finger_inertia_from_csv(finger, finger_index)


data_landmarker = readcell('Mojtaba_model_data_landmarker.csv');
data_inertia = readcell('Mojtaba_model_data_inertia.csv');

landmarker_position = [];
dynpar_mass = [];
dynpar_com = [];
dynpar_inertia = [];

for i = 2:size(data_landmarker,1)
    if contains(data_landmarker{i,1},num2str(finger_index)) && contains(data_landmarker{i,1},{'h'}) ...
            && isnumeric(data_landmarker{i,3})
        landmarker_position = [landmarker_position; ...
        [data_landmarker{i,3},data_landmarker{i,4},data_landmarker{i,5}]*0.001];
    end
end
for i = 2:size(data_inertia,1)
    if contains(data_inertia{i,1},num2str(finger_index)) ...
            && isnumeric(data_inertia{i,2})
        dynpar_mass = [dynpar_mass; data_inertia{i,2}*0.001];
%         dynpar_com_data = [dynpar{i,3},dynpar{i,4},dynpar{i,5}]*0.001;
        dynpar_com = [dynpar_com; ...
            [data_inertia{i,3},data_inertia{i,4},data_inertia{i,5}]*0.001];
        dynpar_inertia = [dynpar_inertia; ...
            [data_inertia{i,6},data_inertia{i,7},data_inertia{i,8}]*1e-9];
    end
end

% update dynamic parameters for PP MP and DP links
for i = 1:3
    finger.list_links(i+1).set_mass(dynpar_mass(i)); % in kg
    finger.list_links(i+1).set_inertia([dynpar_inertia(i,:),zeros(1,3)]);
    finger.list_links(i+1).set_com([finger.list_links(i+1).Length/2,0,0]);
end
finger.update_finger_par_dyn;
end