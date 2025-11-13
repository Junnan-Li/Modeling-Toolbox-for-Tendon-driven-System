% generate an individual function to compute transformation matrix of all frames
% with q as input using par_T representations
% 
% input:
%           par_T:        [4*(nl+nb+nf),4]   par_T parameters of all links
%           par_T_index:  [nb+nf,2]  index of start and end row of each finger
%                               in mdh matrix
%           q_index:    
%           w_T_b:      [4x(nb+nf),4] transformation matrix of each finger base
%           n_links:    [1]     number of base finger of a hand
%           q:          [nj]    joint angle
% 
% Output:   
%           w_T_all_frames: [4,4,nframes] transformation matrix for all frames
%           w_T_all_links:  [4,4,nl]      transformation matrix for all links
% 
% 

function [w_T_all_frames,w_T_all_links] = sim_w_T_all_frames_from_q_par_T(par_T, par_T_index,...
    q_index,w_T_b,n_links,q)
%#codegen

% initialization
nl = length(q);
nb = n_links(1);
nf = n_links(2);
nframe = nl + 2*size(w_T_b,1)/4;
% get all transformation matrix of all links
w_T_b_prior = eye(4);

w_T_all_frames = zeros(4,4,nframe);
w_T_all_links = zeros(4,4,nl);


frame_index = 1;
link_index = 1;
if nb ~= 0
    for i = 1:nb
        par_T_i = par_T(par_T_index(i,1):par_T_index(i,2),:);
        q_i = q(q_index(i,1):q_index(i,2));
        w_T_bi = w_T_b(4*i-3:4*i,:);
        w_T_bi_inhand = w_T_b_prior*w_T_bi;
        w_T_all_frames(:,:,frame_index) = w_T_bi_inhand;
        frame_index = frame_index+1;
        nl_i = length(q_i);

        % update all link properties
        b_T_pre = eye(4);
        for j = 1:nl_i
            % b_T_i = T_mdh_multi(mdh_i(1:j,:));
            b_T_i_wtR = b_T_pre*par_T_i(4*j-3:4*j,:);
            R_q = eul2rotm([0,0,q_i(j)],'XYZ');
                    % default rotation around z axis
            T_q_i = [R_q,zeros(3,1);[0,0,0,1]];
            b_T_i = b_T_i_wtR*T_q_i;
            b_T_pre = b_T_i;
            w_T_all_frames(:,:,frame_index) = w_T_bi_inhand*b_T_i;
            w_T_all_links(:,:,link_index) = w_T_all_frames(:,:,frame_index);
            frame_index = frame_index+1;
            link_index = link_index+1;
        end
        % w_T_b_prior is the world frame for the next finger
        w_T_b_prior = w_T_bi_inhand * b_T_pre * par_T_i(end-3:end,:);
        w_T_all_frames(:,:,frame_index) = w_T_b_prior;
        frame_index = frame_index+1;
    end
end
if nf ~= 0
    for i = nb + 1 : nb + nf
        par_T_i = par_T(par_T_index(i,1):par_T_index(i,2),:);
        q_i = q(q_index(i,1):q_index(i,2));
        w_T_bi = w_T_b(4*i-3:4*i,:);
        w_T_bi_inhand = w_T_b_prior * w_T_bi;
        w_T_all_frames(:,:,frame_index) = w_T_bi_inhand;
        frame_index = frame_index+1;
        nl_i = length(q_i);
        % update all link properties
        b_T_pre = eye(4);
        for j = 1:nl_i
            b_T_i_wtR = b_T_pre*par_T_i(4*j-3:4*j,:);
            R_q = eul2rotm([0,0,q_i(j)],'XYZ');
                    % default rotation around z axis
            T_q_i = [R_q,zeros(3,1);[0,0,0,1]];
            b_T_i = b_T_i_wtR*T_q_i;
            b_T_pre = b_T_i;
            w_T_all_frames(:,:,frame_index) = w_T_bi_inhand*b_T_i;
            w_T_all_links(:,:,link_index) = w_T_all_frames(:,:,frame_index);
            frame_index = frame_index+1;
            link_index = link_index+1;
        end
        w_T_ee_i = w_T_bi_inhand * b_T_pre * par_T_i(end-3:end,:);
        w_T_all_frames(:,:,frame_index) = w_T_ee_i;
        frame_index = frame_index+1;
    end
end

end