function fct_plot_boxes(s_plot_edges_out, s_mp_in)
% Alexander Toedtheide (c) 2019


nop = 1;
TF_fields = fieldnames(s_plot_edges_out);
for i=1:length(TF_fields)
    bodies = fieldnames(s_plot_edges_out.(TF_fields{i}).body_data);
    for j = 1:length(bodies)
        %%%%%%%%%%%%%%%%%%%% hier
        r_box_edges_0_W = s_plot_edges_out(nop).(TF_fields{i}).body_data.(bodies{j}).r_box_edges_0_W;
        
              fac = [1 2 4 3; ...
        4 3 7 8; ...
        1 2 6 5; ...
        2 6 8 4; ...
        5 6 8 7; ...
        1 5 7 3];
        patch('Faces',fac ,'Vertices',r_box_edges_0_W(:,1:3), ...
              'FaceColor',s_mp_in.(TF_fields{i}).body_data.(bodies{j}).color_, ...
              'FaceAlpha',.1); hold on; % patch function
        

        
    end


end