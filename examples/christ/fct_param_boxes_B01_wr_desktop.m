function s_mp = fct_param_boxes_B01_wr_desktop()
% Alexander Toedtheide Feb 2020


s_mp.T_0_0.body_data.B1.r_com_KS_MDH = [-0.15 -0.1 -0.1];
s_mp.T_0_0.body_data.B1.l_box = [0.05 0.3   0.3 ];
s_mp.T_0_0.body_data.B1.density = 2700;
s_mp.T_0_0.body_data.B1.color_ = 'y';


%%
s_mp.T_0_sh1.body_data.B1.r_com_KS_MDH = [0 0.16 0.05];
s_mp.T_0_sh1.body_data.B1.l_box = [0.025 0.025 0.18];
s_mp.T_0_sh1.body_data.B1.density = 2700;
s_mp.T_0_sh1.body_data.B1.color_ = 'k';

s_mp.T_0_sh1.body_data.B2.r_com_KS_MDH = [0 0.075 0.15];
s_mp.T_0_sh1.body_data.B2.l_box = [00.025 .18  0.025];
s_mp.T_0_sh1.body_data.B2.density = 2700;
s_mp.T_0_sh1.body_data.B2.color_ = 'k';

%%%

s_mp.T_0_sh2.body_data.B1.r_com_KS_MDH = [0 0 0.2];
s_mp.T_0_sh2.body_data.B1.l_box = [0.025 0.2 0.025];
s_mp.T_0_sh2.body_data.B1.density = 2700;
s_mp.T_0_sh2.body_data.B1.color_ = 'g';

s_mp.T_0_sh2.body_data.B2.r_com_KS_MDH = [0 0.1 0.1];
s_mp.T_0_sh2.body_data.B2.l_box = [0.02 0.025 0.2];
s_mp.T_0_sh2.body_data.B2.density = 2700;
s_mp.T_0_sh2.body_data.B2.color_ = 'g';


%%

s_mp.T_0_sh3.body_data.B1.r_com_KS_MDH = [0.15 0 0.11];
s_mp.T_0_sh3.body_data.B1.l_box = [0.35 0.035 0.02];
s_mp.T_0_sh3.body_data.B1.density = 2700;
s_mp.T_0_sh3.body_data.B1.color_ = 'b';

s_mp.T_0_sh3.body_data.B2.r_com_KS_MDH = [0.3 0 0.075];
s_mp.T_0_sh3.body_data.B2.l_box = [0.035 0.035 0.1];
s_mp.T_0_sh3.body_data.B2.density = 2700;
s_mp.T_0_sh3.body_data.B2.color_ = 'b';

% s_mp.T_0_sh3.body_data.B2.r_com_KS_MDH = [0.5 0 0.075];
% s_mp.T_0_sh3.body_data.B2.l_box = [0.06 0.06 0.05];
% s_mp.T_0_sh3.body_data.B2.density = 2700;
% s_mp.T_0_sh3.body_data.B2.color_ = 'b';

% s_mp.T_0_sh3.body_data.B3.r_com_KS_MDH = [0 0 0.05];
% s_mp.T_0_sh3.body_data.B3.l_box = [0.075 0.075 0.05];
% s_mp.T_0_sh3.body_data.B3.density = 2700;
% s_mp.T_0_sh3.body_data.B3.color_ = 'b';
 
s_mp.T_0_el.body_data.B1.r_com_KS_MDH = [0 -0.025 0];
s_mp.T_0_el.body_data.B1.l_box = [0.06 0.09 0.06];
s_mp.T_0_el.body_data.B1.density = 2700;
s_mp.T_0_el.color_ = 'r';
s_mp.T_0_el.body_data.B1.color_ = 'r';

s_mp.T_0_fa.body_data.B1.r_com_KS_MDH = [0 0 -0.13];
s_mp.T_0_fa.body_data.B1.l_box = [0.06 0.06 0.10];
s_mp.T_0_fa.body_data.B1.density = 2700;
s_mp.T_0_fa.body_data.B1.color_ = 'g';

s_mp.T_0_fa.body_data.B2.r_com_KS_MDH = [0 0 -0.05];
s_mp.T_0_fa.body_data.B2.l_box = [0.03 0.03 0.08];
s_mp.T_0_fa.body_data.B2.density = 2700;
s_mp.T_0_fa.body_data.B2.color_ = 'g';

s_mp.T_0_wj1.body_data.B1.r_com_KS_MDH = [0 0 0];
s_mp.T_0_wj1.body_data.B1.l_box = [0.01 0.03 0.01];
s_mp.T_0_wj1.body_data.B1.density = 2700;
s_mp.T_0_wj1.body_data.B1.color_ = 'y';

s_mp.T_0_wj2.body_data.B1.r_com_KS_MDH = [0.025 0 0];
s_mp.T_0_wj2.body_data.B1.l_box = [0.02 0.06 0.06];
s_mp.T_0_wj2.body_data.B1.density = 1000;
s_mp.T_0_wj2.body_data.B1.color_ = 'b';

% s_mp.T_0_wj2.body_data.B2.r_com_KS_MDH = [0.1 0.055 0.075];
% s_mp.T_0_wj2.body_data.B2.l_box = [0.07 0.07 0.3];
% s_mp.T_0_wj2.body_data.B2.density = 1000;
% s_mp.T_0_wj2.body_data.B2.color_ = 'y';

end