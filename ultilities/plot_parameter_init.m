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

end