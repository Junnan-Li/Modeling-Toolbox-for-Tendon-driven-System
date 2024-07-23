% calculate the obstacle via points coordinates 
% 
% Input:
% Output:
% 
% 
% Reference:
%   [1] Garner, B. A., & Pandy, M. G. (2000). The obstacle-set method for representing 
%   muscle paths in musculoskeletal models. Computer Methods in Biomechanics and
%   Biomedical Engineering, 3(1), 1–30. https://doi.org/10.1080/10255840008915251
% 
% 
% Junnan Li, junnan.li@tum.de, 07.2024


function [l_QT,w_status,w_Q_p,w_T_p] = cal_obstacle_vp_cyl_Garner(w_T_obs, w_P_p, w_S_p, radius, wrap_direction )

obs_P_p = T_p31(inv(w_T_obs),w_P_p);
obs_S_p = T_p31(inv(w_T_obs),w_S_p);

% Step 2: compute Q and T
if wrap_direction
    obs_Q_x = (obs_P_p(1)*radius^2-radius*obs_P_p(2)*sqrt(obs_P_p(1)^2+obs_P_p(2)^2-radius^2))...
        /(obs_P_p(1)^2+obs_P_p(2)^2);

    obs_Q_y = (obs_P_p(2)*radius^2+radius*obs_P_p(1)*sqrt(obs_P_p(1)^2+obs_P_p(2)^2-radius^2))...
        /(obs_P_p(1)^2+obs_P_p(2)^2);

    obs_T_x = (obs_S_p(1)*radius^2+radius*obs_S_p(2)*sqrt(obs_S_p(1)^2+obs_S_p(2)^2-radius^2))...
        /(obs_S_p(1)^2+obs_S_p(2)^2);

    obs_T_y = (obs_S_p(2)*radius^2-radius*obs_S_p(1)*sqrt(obs_S_p(1)^2+obs_S_p(2)^2-radius^2))...
        /(obs_S_p(1)^2+obs_S_p(2)^2);

else
    obs_Q_x = (obs_P_p(1)*radius^2+radius*obs_P_p(2)*sqrt(obs_P_p(1)^2+obs_P_p(2)^2-radius^2))...
        /(obs_P_p(1)^2+obs_P_p(2)^2);

    obs_Q_y = (obs_P_p(2)*radius^2-radius*obs_P_p(1)*sqrt(obs_P_p(1)^2+obs_P_p(2)^2-radius^2))...
        /(obs_P_p(1)^2+obs_P_p(2)^2);

    obs_T_x = (obs_S_p(1)*radius^2-radius*obs_S_p(2)*sqrt(obs_S_p(1)^2+obs_S_p(2)^2-radius^2))...
        /(obs_S_p(1)^2+obs_S_p(2)^2);

    obs_T_y = (obs_S_p(2)*radius^2+radius*obs_S_p(1)*sqrt(obs_S_p(1)^2+obs_S_p(2)^2-radius^2))...
        /(obs_S_p(1)^2+obs_S_p(2)^2);
end
l_QT_xy = abs(radius*acos(1-((obs_Q_x-obs_T_x)^2+(obs_Q_y-obs_T_y)^2)/(2*radius^2)));

obs_Q_z = obs_P_p(3)+((obs_S_p(3)-obs_P_p(3))*norm([obs_Q_x-obs_P_p(1);obs_Q_y-obs_P_p(2)])) ...
    /(norm([obs_Q_x-obs_P_p(1);obs_Q_y-obs_P_p(2)])+l_QT_xy+norm([obs_T_x-obs_S_p(1);obs_T_y-obs_S_p(2)]));
obs_T_z = obs_S_p(3)-((obs_S_p(3)-obs_P_p(3))*norm([obs_T_x-obs_S_p(1);obs_T_y-obs_S_p(2)])) ...
    /(norm([obs_Q_x-obs_P_p(1);obs_Q_y-obs_P_p(2)])+l_QT_xy+norm([obs_T_x-obs_S_p(1);obs_T_y-obs_S_p(2)]));

obs_Q_p = [obs_Q_x;obs_Q_y;obs_Q_z];
obs_T_p = [obs_T_x;obs_T_y;obs_T_z];


w_Q_p = T_p31(w_T_obs,obs_Q_p);
w_T_p = T_p31(w_T_obs,obs_T_p);

% Step 3: Wrapping condition

det_qt = (obs_Q_x*obs_T_y - obs_Q_y*obs_T_x);
if det_qt < 0 
    w_status = 0;

else
    w_status = 1;
end

% Step 4: Muscle length
if w_status
    l_QT = sqrt(l_QT_xy^2 + (obs_Q_z-obs_T_z)^2);
else
    l_QT = norm(obs_P_p-obs_S_p);
end