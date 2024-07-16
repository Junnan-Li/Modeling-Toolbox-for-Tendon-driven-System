% plot the landmarkers

function plot_Mojtaba_landmarkers(varargin)

data_landmarker = readcell('Mojtaba_model_data_landmarker.csv');

markersize = 20;

if nargin == 0
    for i = 2:size(data_landmarker,1)
        if isnumeric(data_landmarker{i,3})
            plot3(data_landmarker{i,3}* 0.001,data_landmarker{i,4}* 0.001,data_landmarker{i,5}* 0.001, ...
                '.','MarkerSize',markersize,'Color','k');
            hold on
        end

    end
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
elseif nargin == 1
    keywords = varargin{1};
    for i = 2:size(data_landmarker,1)
        if contains(data_landmarker{i,1},keywords) && contains(data_landmarker{i,1},{'h','b'}) ...
                && isnumeric(data_landmarker{i,3})
            plot3(data_landmarker{i,3}* 0.001,data_landmarker{i,4}* 0.001,data_landmarker{i,5}* 0.001, ...
                '.','MarkerSize',markersize,'Color','k');
            hold on
        end

    end
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
else
    disp('[plot_Mojtaba_landmarkers]: too much input!')
end
end