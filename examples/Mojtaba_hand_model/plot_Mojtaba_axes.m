% plot the axis

function plot_Mojtaba_axes(varargin)

data_axes = readcell('Mojtaba_model_data_axes.csv');
markersize = 20;

if nargin == 0
    for i = 2:size(data_axes,1)
        if isnumeric(data_axes{i,3})
            plot3([data_axes{i,7}+10*data_axes{i,3};data_axes{i,7} - 10*data_axes{i,3}]* 0.001,...
                [data_axes{i,8}+10*data_axes{i,4};data_axes{i,8} - 10*data_axes{i,4}]* 0.001,...
                [data_axes{i,9}+10*data_axes{i,5};data_axes{i,9} - 10*data_axes{i,5}]* 0.001,...
                '--','LineWidth',2, 'Color','k');
            hold on
        end

    end
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
elseif nargin == 1
    keywords = varargin{1};
    for i = 2:size(data_axes,1)
        if contains(data_axes{i,1},keywords) && isnumeric(data_axes{i,3})
            plot3([data_axes{i,7}+10*data_axes{i,3};data_axes{i,7} - 10*data_axes{i,3}]* 0.001,...
                [data_axes{i,8}+10*data_axes{i,4};data_axes{i,8} - 10*data_axes{i,4}]* 0.001,...
                [data_axes{i,9}+10*data_axes{i,5};data_axes{i,9} - 10*data_axes{i,5}]* 0.001,...
                '--','LineWidth',2, 'Color','k');
            hold on
        end

    end
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
else
    disp('[plot_Mojtaba_axes]: too much input!')
end
end