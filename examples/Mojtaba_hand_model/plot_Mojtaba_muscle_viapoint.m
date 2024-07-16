% plot the muscle path and distinguish the origin, viapoints, and insertion


function plot_Mojtaba_muscle_viapoint(varargin)


data_muscle = readcell('Mojtaba_model_data_muscle_path.csv');

markersize = 20;


if nargin == 0
    for i = 4:size(data_muscle,1)
        if ~ismissing(data_muscle{i,2})
            if isnumeric(data_muscle{i,5}) 
                if contains(data_muscle{i,3},'O')
                    plot3(data_muscle{i,5}* 0.001,data_muscle{i,6}* 0.001,data_muscle{i,7}* 0.001, ...
                        'x','MarkerSize',markersize, 'Color','r');
                    hold on
                elseif contains(data_muscle{i,3},'I')
                    plot3(data_muscle{i,5}* 0.001,data_muscle{i,6}* 0.001,data_muscle{i,7}* 0.001, ...
                        '*','MarkerSize',markersize, 'Color','r');
                    hold on
                elseif contains(data_muscle{i,3},'V')
                    plot3(data_muscle{i,5}* 0.001,data_muscle{i,6}* 0.001,data_muscle{i,7}* 0.001, ...
                        '.','MarkerSize',markersize, 'Color','r');
                    hold on
                else
                    plot3(data_muscle{i,5}* 0.001,data_muscle{i,6}* 0.001,data_muscle{i,7}* 0.001, ...
                        'square','MarkerSize',markersize, 'Color','r');
                    hold on
                end
            end
        end
    end
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
elseif nargin == 1
    muscle_list = varargin{1};
    for muscle_i = 1:length(muscle_list)
        for i = 4:size(data_muscle,1)
            if ~ismissing(data_muscle{i,2})
                if contains(data_muscle{i,2},muscle_list{muscle_i}) && isnumeric(data_muscle{i,5})
                    if contains(data_muscle{i,3},'O')
                        plot3(data_muscle{i,5}* 0.001,data_muscle{i,6}* 0.001,data_muscle{i,7}* 0.001, ...
                            'x','MarkerSize',markersize, 'Color','r');
                        hold on
                    elseif contains(data_muscle{i,3},'I')
                        plot3(data_muscle{i,5}* 0.001,data_muscle{i,6}* 0.001,data_muscle{i,7}* 0.001, ...
                            '*','MarkerSize',markersize, 'Color','r');
                        hold on
                    elseif contains(data_muscle{i,3},'V')
                        plot3(data_muscle{i,5}* 0.001,data_muscle{i,6}* 0.001,data_muscle{i,7}* 0.001, ...
                            '.','MarkerSize',markersize, 'Color','r');
                        hold on
                    else
                        plot3(data_muscle{i,5}* 0.001,data_muscle{i,6}* 0.001,data_muscle{i,7}* 0.001, ...
                            'square','MarkerSize',markersize, 'Color','r');
                        hold on
                    end
                end
            end
        end
    end
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
else
    disp('[plot_Mojtaba_muscle_viapoint]: too much input!')
end
end