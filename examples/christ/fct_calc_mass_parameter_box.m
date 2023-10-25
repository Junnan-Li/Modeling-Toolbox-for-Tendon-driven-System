function s_mp_in = fct_calc_mass_parameter_box(s_mp_in)
% Alexander Toedtheide (c) 2019

% calculate mass properties based on boxed which are distributed over the
% structure


TF_fields = fieldnames(s_mp_in);

for i=1:length(TF_fields)
    bodies = fieldnames(s_mp_in.(TF_fields{i}).body_data);
    for j = 1:length(bodies)
      
        % calulcate mechanical properties for every body in every TF
        [I_KS_MDH_KS_MDH, I_SP_KS_MDH, r_box_edges_0, m_box] = mech_prop_box(s_mp_in.(TF_fields{i}).body_data.(bodies{j}).r_com_KS_MDH, ...
            s_mp_in.(TF_fields{i}).body_data.(bodies{j}).l_box, ...
            s_mp_in.(TF_fields{i}).body_data.(bodies{j}).density);
        
        
        s_mp_in.(TF_fields{i}).body_data.(bodies{j}).r_box_edges_0 = r_box_edges_0;
        s_mp_in.(TF_fields{i}).body_data.(bodies{j}).I_KS_MDH_KS_MDH = I_KS_MDH_KS_MDH;
        s_mp_in.(TF_fields{i}).body_data.(bodies{j}).I_SP_KS_MDH = I_SP_KS_MDH;
        s_mp_in.(TF_fields{i}).body_data.(bodies{j}).m_box = m_box;
        
    end
    
    % fuse all mechanical properties inside every TF
    [r_com_Fuse, I_KS_MDH_KS_MDH_FUSE, I_SP_KS_MDH_FUSE, m_Fuse] = fuse_mass_properties(s_mp_in.(TF_fields{i}).body_data);
    s_mp_in.(TF_fields{i}).r_com_Fuse = r_com_Fuse;
    s_mp_in.(TF_fields{i}).I_KS_MDH_KS_MDH_FUSE = I_KS_MDH_KS_MDH_FUSE;
    s_mp_in.(TF_fields{i}).I_SP_KS_MDH_FUSE = I_SP_KS_MDH_FUSE;
    s_mp_in.(TF_fields{i}).m_Fuse = m_Fuse;
    

end

a = 1;



end

function [r_com_Fuse, I_KS_MDH_KS_MDH_FUSE, I_SP_KS_MDH_FUSE, m_Fuse] = fuse_mass_properties(s_mp_TF)

a = 1;
r_com_Fuse = zeros(3,1);
I_SP_KS_MDH_FUSE = zeros(3,3);
I_KS_MDH_KS_MDH_FUSE = zeros(3,3);
m_Fuse = 0;

bodies = fieldnames(s_mp_TF);

for i = 1:length(bodies)
    m_Fuse = m_Fuse + s_mp_TF.(bodies{i}).m_box;
end


for i = 1:length(bodies)
    r_box_single = s_mp_TF.(bodies{i}).r_com_KS_MDH;
    m_box_single = s_mp_TF.(bodies{i}).m_box;
    r_com_Fuse = r_com_Fuse + ((r_box_single*m_box_single)/m_Fuse)';
    I_SP_KS_MDH_FUSE = I_SP_KS_MDH_FUSE + s_mp_TF.(bodies{i}).I_SP_KS_MDH;
    I_KS_MDH_KS_MDH_FUSE = I_KS_MDH_KS_MDH_FUSE + s_mp_TF.(bodies{i}).I_KS_MDH_KS_MDH;
end


end


function [I_KS_MDH_KS_MDH, I_SP_KS_MDH, r_box_edges_0, m_box] = mech_prop_box(r_com_KS_MDH, l_box, density)


%% calc position and size of box
% r_com_KS_MDH = [0.1; 0.2; 0.3];
% l_x_KS_MDH = 0.1;
% l_y_KS_MDH = 0.2;
% l_z_KS_MDH = 0.3;

% % m_KS_MDH
% l_box = [l_x_KS_MDH, l_y_KS_MDH, l_z_KS_MDH];
l_x_KS_MDH = l_box(1);
l_y_KS_MDH = l_box(2);
l_z_KS_MDH = l_box(3);

m_box = l_x_KS_MDH*l_y_KS_MDH*l_z_KS_MDH*density;

n = 1;
for i = -1:2:2
    for j = -1:2:2
        for k = -1:2:2
            r_box_edges_0(n,:) = [r_com_KS_MDH(1)+i*l_box(1)/2, ...
                r_com_KS_MDH(2)+j*l_box(2)/2, ...
                r_com_KS_MDH(3)+k*l_box(3)/2];
            n = n +1;
        end
    end
end

fac = [1 2 4 3; ...
    4 3 7 8; ...
    1 2 6 5; ...
    2 6 8 4; ...
    5 6 8 7; ...
    1 5 7 3];


%     for iii = 1:size(vert_w,1)
%         vert(iii,:)=(TF.KSW.T_0_EE*vert_w(iii,:)');
%         vert_(iii,1:3) = vert(iii,1:3);
%     end
%     patch('Faces',fac,'Vertices',r_box_edges_0,'FaceColor','b', 'FaceAlpha',.1);  % patch function
% axis equal;

I_SP_x_KS_MDH = (1/12)*m_box*(l_y_KS_MDH^2 + l_z_KS_MDH^2);
I_SP_y_KS_MDH = (1/12)*m_box*(l_x_KS_MDH^2 + l_z_KS_MDH^2);
I_SP_z_KS_MDH = (1/12)*m_box*(l_x_KS_MDH^2 + l_y_KS_MDH^2);

I_SP_KS_MDH = zeros(3,3);
I_SP_KS_MDH(1,1) = I_SP_x_KS_MDH;
I_SP_KS_MDH(2,2) = I_SP_y_KS_MDH;
I_SP_KS_MDH(3,3) = I_SP_z_KS_MDH;

% I_KS_MDH_KS_MDH = I_SP_KS_MDH + m_box*r_com_KS_MDH'*r_com_KS_MDH;
I_KS_MDH_KS_MDH = inertia_steiner(I_SP_KS_MDH, r_com_KS_MDH', m_box);

end
%
%
% function mech_prop_cylinder(r_com_KS_MDH, l_cyl, R_cyl, m)
%
% n = 1;
% rotation_ = 1;
% sign_vector = [1 1 -1 -1];
% save_sign_vector = 1;
%
% while rotation_ <= 20
%     for i = 1:4
%         if sign(sign_vector(i)) == save_sign_vector
%             rotation_ = rotation_ + 1;
%         end
%
%         r_cyl_edges_0(n,:) = [r_com_KS_MDH + [0; 0; sign_vector(i)*l_cyl/2] + [R_cyl*cos(1/20*2*pi)*rotation_; R_cyl*sin(1/20*2*pi)*rotation_; 0]]'
%
%         if i == 4
%             r_cyl_edges_0(n,1:2) =  r_cyl_edges_0(n-3,1:2);
%         end
%
%         n = n +1;
%         save_sign_vector = sign_vector(i);
%     end
% end
%
% for j = 1:size(r_cyl_edges_0, 1)
%    if j <= size(r_cyl_edges_0, 1)
%    fac_(j,:) = [1, 2, 3, 4] +(j-1)*2
%    else
%    fac_(j,:) = [j-1, j, 1, 2]
%    end
% end
% figure; hold on;
%
% patch('Faces',fac_(1,:),'Vertices',r_cyl_edges_0(1:4,:), 'FaceColor','b', 'FaceAlpha',.1);  % patch function
% % axis equal;
%
% end