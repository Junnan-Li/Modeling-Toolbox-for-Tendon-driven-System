%  read from whole_hand_003_processed.csv
%  marker index:
%       1: MP5
%       2: MP4
%       3. MP2
%       4. MCP3
%       5. DP2
%       6. DP3
%       7. PP2
%       8. MCP2
%       9. PP1
%       10. MCP5
%       11. MCP1
%       12. CMC1
%       13. MC2
%       14. MC4
%       15. MCP4
%       16. PP5
%       17. PP3
%       18. PP4
%       19. DP5
%       20. DP1
%       21. MP3
%       22. DP4
%       
%   MC: [13,14]
% % index finger: [8,7,3,5]
%       


clear all
close all
clc


%%

data = table2cell(readtable('whole_hand_003_processed.csv'));


%% define origin
marker_pos_index = [];
marker_index = 13;
for i = 5:str2num(data{end,1})
    x = str2num(data{i,3*marker_index});
    y = str2num(data{i,3*marker_index+1});
    z = str2num(data{i,3*marker_index+2});
    marker_pos_index = [marker_pos_index;x,y,z];
end
% take pos of MC2, MC4, MCP3
% MC2
init_pos_13 = [str2num(data{5,3*13}),str2num(data{5,3*13+1}),str2num(data{5,3*13+2})];
% MC4
init_pos_14 = [str2num(data{5,3*14}),str2num(data{5,3*14+1}),str2num(data{5,3*14+2})];
% MCP2
init_pos_8 = [str2num(data{5,3*8}),str2num(data{5,3*8+1}),str2num(data{5,3*8+2})];

y_axis = (init_pos_14-init_pos_13)/norm(init_pos_14-init_pos_13);
z_axis = cross(init_pos_8-init_pos_13,y_axis)/norm(cross(init_pos_8-init_pos_13,y_axis));
x_axis = cross(y_axis,z_axis);
R = [x_axis',y_axis',z_axis'];
origin = (init_pos_13 + init_pos_14)'/2;

%% lowpass filter
fc = 6;
fs = 500;
[b,a] = butter(4,fc/(fs/2),'low');


%% individual finger
f2 = [13,14,8,7,3,5];
num_m = length(f2);
marker_pos = [];
for i = 5:str2num(data{end,1})
    marker_pos_i = [];
    for j = 1:length(f2)
        x = str2num(data{i,3*f2(j)});
        y = str2num(data{i,3*f2(j)+1});
        z = str2num(data{i,3*f2(j)+2});
        pos_i = [x,y,z];
        pos_i_trans = R'*(pos_i'-origin);
        marker_pos_i = [marker_pos_i,pos_i_trans'];  
    end
    marker_pos = [marker_pos;marker_pos_i]; 
end
% filter 
marker_pos_fil = 0*marker_pos;
for i = 1:size(marker_pos,2)
    marker_pos_fil(:,i) = filtfilt(b,a,marker_pos(:,i));
end

%% plot
time_step = size(marker_pos_fil,1);
% mi = 22;
figure(1)
for i = 1:time_step/10
    for j = 1:num_m
        plot3(marker_pos_fil(10*i,3*j-2),marker_pos_fil(10*i,3*j-1),marker_pos_fil(10*i,3*j),...
            'r.','MarkerSize',25);
        hold on
    
    end
    
    grid on
    axis equal
    xlabel("x");
    ylabel("y");
    zlabel("z");
    xlim([-50 100])
    ylim([-100 100])
    zlim([-100 60])

    drawnow
    pause(0.002)
    hold off

    fprintf('time: %.3f \n', str2num(data{5+10*i,2}));
end

