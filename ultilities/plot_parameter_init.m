% plot setting for Finger and Hand 
% 
% entities:
%       inhand:         [binary] 0: plot as Finger object alone; 1: plot as Finger in
%                       hand object. different w_T_base/_inhand


function parstr = plot_parameter_init()
% init the parameter struct for plot_finger
parstr = struct();

parstr.inhand = 0;

parstr.linecolor = 'r';
parstr.linewidth = 3;
parstr.markersize = 5;
parstr.markercolor= 'g';
parstr.axis_show = 1;
parstr.axis_len = 0.1;
parstr.linecolor = 'r';

parstr.com_marker = '*';
parstr.com_markersize = 5;
parstr.com_markercolor = 'r';

parstr.viapoint_marker = '.';
parstr.viapoint_markersize = 25;
parstr.viapoint_markercolor = 'g';

parstr.muscle_plot_obstacleset = false;
parstr.muscle_linecolor = 'm';
parstr.muscle_linewidth = 2;
parstr.muscle_markersize = 15;


parstr.obstacle_color = 'g';
parstr.obstacle_alpha = 0.2;
end