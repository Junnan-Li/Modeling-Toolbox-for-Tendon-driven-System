function s_plot_edges_out = fct_calc_box_edges(s_mp, s_plot_box_edges)
% Alexander Toedtheide (c) 2019

%% initalizse struct
TF_fields = fieldnames(s_mp);
for i=1:length(TF_fields)
    bodies = fieldnames(s_mp.(TF_fields{i}).body_data);
    for j = 1:length(bodies)
        s_plot_edges_.(TF_fields{i}).body_data.(bodies{j}).r_box_edges_0_W = zeros(8,4);
    end
end

number_points = size(s_plot_box_edges.TF.KSW.T_W_0.data,3);
s_plot_edges(1:number_points) = s_plot_edges_;

%% calculate box point in W system
for nop = 1:number_points
TF_fields = fieldnames(s_mp);
for i=1:length(TF_fields)
    bodies = fieldnames(s_mp.(TF_fields{i}).body_data);
    for j = 1:length(bodies)
        %%%%%%%%%%%%%%%%%%%% hier
        r_box_edges_0_W = zeros(8, 4);
        T_W_0 = s_plot_box_edges.TF.KSW.T_W_0.data(:,:, nop);
        T_0_i = s_plot_box_edges.TF.KS0.(TF_fields{i}).data(:,:, nop);
        r_box_edges_0_ = s_mp.(TF_fields{i}).body_data.(bodies{j}).r_box_edges_0;
        for k = 1:8   
            r_box_edges_0_W(k,:) = (T_W_0*T_0_i*[r_box_edges_0_(k,:), 1]')';
        end
        s_plot_edges_out(nop).(TF_fields{i}).body_data.(bodies{j}).r_box_edges_0_W = r_box_edges_0_W;
        
    end
  
end
end



end
% end
