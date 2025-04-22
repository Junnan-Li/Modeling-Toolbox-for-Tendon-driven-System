% recursive Newton-Euler method to compute inverse dynamic with mdh
% only end-effector external force input
% parameters
% Input:
%       T:                [4,4, n_frame] each Finger object has nqi+2  
%       qD,qDD:           [n_q] joint states
%       n_links
%       q_index
%       Mass:             [n_q+(nb+nf)] mass vector. the first mass is the base 
%       X_base:           [6] eulerxyz
%       XD_base:          [6] translational and angular velocity
%       F_ext:            [6,nf] force/moment that exerted by the environment to the endeffector frame (each finger). 
%       CoM:              [3,n_q+1] in local frame; last column is endeffector
%       I:                [6,n_q+1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
%       g:                [3] gravity term in world frame

% 
% Output:
%     Tau:              [n_q] joint torque 
%     F:                [6,n_q+2] force/moment vectors that the i link exert to the i+1 link

% internal variable:
%   n_q: number of joints
%   n_b = kin_str.nb; % number of bases
%   n_f = kin_str.nf; % number of fingers
%   njb = kin_str.njb; % number of joints of each base
%   njf = kin_str.njf; % number of joints of each finger
%   n_frame = n_q + 2*(n_b+n_f); % each Finger has nqi + 2 frames (except world frame) 
%   V: [6xn_frame] linear & angular velocity of each frames
%   VD: [6xn_frame] linear & angular accelerations of each frames
%   VD_c: [6xn_frame] linear & angular accelerations of each center of mass;
%         (default endeffector center of mass is the origin og the frame)
%   
% 
% Frames:
%   0:World, 1:Base, 2:F_1, 3:F_2,..., end:F_endeffector
% 
% source:
%   [1] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, Robotics: Modeling, Planning, and Control, vol. 16, no. 4. 2009.
% 
% Comment: [04/23] XDbase not updated (VD_c)


function [Tau,F] = invdyn_ne_T(T,qD,qDD, n_links, q_index, Mass, XD_base,XDD_base, F_ext, CoM, I, g)
%#codegen

% test input dimension

n_b = n_links(1); % number of bases
n_f = n_links(2); % number of fingers
njb = q_index(1:n_b,2)-q_index(1:n_b,1) + 1; % number of joints of each base
njf = q_index((1+n_b):(n_b+n_f),2)-q_index((1+n_b):(n_b+n_f),1) + 1; % number of joints of each finger
n_q = length(qD);
n_frame = n_q + 2*(n_b+n_f); % each Finger has nqi + 2 frames (except world frame) 

assert(n_b == length(njb) && n_f == length(njf),'[invdyn_ne_T] kin_str input error!');
% assert(n_q == length(qD) && n_q == length(qDD),'[invdyn_ne_T] dimension error!');
% assert(all(size(F_ext) == [6,n_f]),'[invdyn_ne_T] F_ext dimension error!');
assert(all(size(T) == [4,4,n_frame]),'[invdyn_ne_T] T dimension error!');

% F (force and moment) with respect to the Wolrd Frame
% dimension: [6xn_frame]; 1st column is base
Tau = zeros(length(qD),1);
F = zeros(6,n_frame);
% F(:,end) = -F_ext;

% initializing
V = zeros(6,n_frame);
VD = zeros(6,n_frame);
Mass_n_frame = zeros(n_frame,1);
CoM_n_frame = zeros(3,n_frame);
I_n_frame = zeros(6,n_frame);

% updating the base information
V(:,1) = XD_base;
VD(:,1) = XDD_base;
VD_c = VD;
Mass_n_frame(1) = Mass(1);
CoM_n_frame(:,1) = CoM(:,1);
I_n_frame(:,1) = I(:,1);
% update VD_c of base
% W_b_r_c = euler2R_XYZ(X_base(4:6))*CoM(:,1);
W_b_r_c = T(1:3,1:3,1)*CoM(:,1);
VD_c(1:3,1) = VD(1:3,1) + cross(VD(4:6,1),W_b_r_c) + cross(V(4:6,1),cross(V(4:6,1),W_b_r_c));

i_frame = 1;
i_q_start = 0;
i_q_end = 0;
i_com_start = 0;
i_com_end = 0;

% frame index of Base&Finger start and end
i_frame_base_start = zeros(n_b,1);
i_frame_base_end = zeros(n_b,1);
i_frame_finger_start = zeros(n_f,1);
i_frame_finger_end = zeros(n_f,1);
i_frame_q = zeros(n_q,1);

% initialize for simulink
W_i_r_c = nan(3,1);

%% forward recursion: 
% for bases
if n_b ~= 0
    for base_i = 1:n_b
        i_frame_base_start(base_i) = i_frame;
        i_q_start = i_q_end+1;
        i_q_end = i_q_start + njb(base_i)-1;
        i_com_start = i_com_end+1;
        i_com_end = i_com_start + njb(base_i);
        for i = 1:njb(base_i)+2
            % start from the first link of base
            % the indexes of current frame
            i_q = i_q_start+i-2;
            i_com = i_com_start +i-1;
            if i_frame == 1
                % the first base is already updated
                i_frame = i_frame+1;
                continue
            end
            %         Mass_n_frame(i_frame) = Mass(i_com);
            %         CoM_n_frame(:,i_frame) = CoM(:,i_com);
            %         I_n_frame(:,i_frame) = I(:,i_com);
            if i == 1
                % the base, without joint
                qD_i = 0;
                qDD_i = 0;
                CoM_i = CoM(:,i_com);
                Mass_n_frame(i_frame) = Mass(i_com);
                CoM_n_frame(:,i_frame) = CoM(:,i_com);
                I_n_frame(:,i_frame) = I(:,i_com);
            elseif i == njb(base_i)+2
                % the endeffector, no joint, no links, only frame
                qD_i = 0;
                qDD_i = 0;
                CoM_i = [0;0;0]; % virtual endeffector link
                Mass_n_frame(i_frame) = 0;
                CoM_n_frame(:,i_frame) = CoM_i;
                I_n_frame(:,i_frame) = zeros(6,1);
            else
                %         q_i = q(i-1);
                qD_i = qD(i_q);
                qDD_i = qDD(i_q);
                CoM_i = CoM(:,i_com);
                i_frame_q(i_q) = i_frame;
                Mass_n_frame(i_frame) = Mass(i_com);
                CoM_n_frame(:,i_frame) = CoM(:,i_com);
                I_n_frame(:,i_frame) = I(:,i_com);
            end
            % frame qi to World
            W_T_i = T(:,:,i_frame);
            W_R_i = W_T_i(1:3,1:3);
            W_T_i_1 = T(:,:,i_frame-1);
            i_1_T_i =  W_T_i_1\W_T_i;

            W_i_1_r_i = W_T_i_1(1:3,1:3)*i_1_T_i(1:3,4);% frame i-1 to frame i
            W_i_r_c = W_R_i*CoM_i;% frame i to center of mass

            % update frame velocity
            V(4:6,i_frame) = V(4:6,i_frame-1) + qD_i*W_R_i*[0 0 1]';
            V(1:3,i_frame) = V(1:3,i_frame-1) + cross(V(4:6,i_frame-1),W_i_1_r_i);
            % update frame acceleration
            VD(4:6,i_frame) = VD(4:6,i_frame-1) + qDD_i*W_R_i*[0 0 1]' + qD_i*cross(V(4:6,i_frame-1),W_R_i*[0 0 1]');
            VD(1:3,i_frame) = VD(1:3,i_frame-1) + cross(VD(4:6,i_frame-1),W_i_1_r_i)+cross(V(4:6,i_frame-1),cross(V(4:6,i_frame-1),W_i_1_r_i));
            % calculate acceleration with respect to the center of mass
            VD_c(4:6,i_frame) = VD(4:6,i_frame);
            VD_c(1:3,i_frame) = VD(1:3,i_frame) + cross(VD(4:6,i_frame),W_i_r_c) + cross(V(4:6,i_frame),cross(V(4:6,i_frame),W_i_r_c));

            i_frame = i_frame+1;
        end
        i_frame_base_end(base_i) = i_frame-1;
    end
    W_T_base_ee = T(:,:,2*n_b + sum(njb));
    V_base_ee = V(:, i_frame-1);
    VD_base_ee = VD(:, i_frame-1);
    % W_T_i
else
    i_frame_base_start = 0;
    i_frame_base_end = 0;
end

% for fingers that share the same endeffector of the lase base
for finger_i = 1:n_f
    i_frame_finger_start(finger_i) = i_frame;
    i_q_start = i_q_end+1;
    i_q_end = i_q_start + njf(finger_i)-1;
    i_com_start = i_com_end+1;
    i_com_end = i_com_start + njf(finger_i);
    for i = 1:njf(finger_i)+2 
        % the indexes of current frame
        i_q = i_q_start+i-2;
        i_com = i_com_start +i-1;
        if i_frame == 1
            % if there is no base, the first base is already updated
            i_frame = i_frame+1;
            continue
        end
        
        if i == 1
            % the base, without joint
            W_T_i_1 = W_T_base_ee;
            qD_i = 0;
            qDD_i = 0;
            CoM_i = CoM(:,i_com);
            Mass_n_frame(i_frame) = Mass(i_com);
            CoM_n_frame(:,i_frame) = CoM(:,i_com);
            I_n_frame(:,i_frame) = I(:,i_com);
            % the V and VD of finger base frame are equal to last base
            % frame
            V_prior = V_base_ee; 
            VD_prior = VD_base_ee;
        elseif i == njf(finger_i)+2
            % the endeffector, no joint, no links, only frame
            W_T_i_1 = T(:,:,i_frame-1);
            qD_i = 0;
            qDD_i = 0;
            CoM_i = [0;0;0]; % virtual endeffector link
            Mass_n_frame(i_frame) = 0;
            CoM_n_frame(:,i_frame) = CoM_i;
            I_n_frame(:,i_frame) = zeros(6,1);
            V_prior = V(:,i_frame-1);
            VD_prior = VD(:,i_frame-1);
        else
            W_T_i_1 = T(:,:,i_frame-1);
            qD_i = qD(i_q);
            qDD_i = qDD(i_q);
            CoM_i = CoM(:,i_com);
            i_frame_q(i_q) = i_frame;
            Mass_n_frame(i_frame) = Mass(i_com);
            CoM_n_frame(:,i_frame) = CoM(:,i_com);
            I_n_frame(:,i_frame) = I(:,i_com);
            V_prior = V(:,i_frame-1);
            VD_prior = VD(:,i_frame-1);
        end
        % frame qi to World
        W_T_i = T(:,:,i_frame);
        W_R_i = W_T_i(1:3,1:3);
        i_1_T_i =  W_T_i_1\W_T_i;

        W_i_1_r_i = W_T_i_1(1:3,1:3)*i_1_T_i(1:3,4);% frame i-1 to frame i
        W_i_r_c = W_R_i*CoM_i;% frame i to center of mass

        % update frame velocity
        V(4:6,i_frame) = V_prior(4:6) + qD_i*W_R_i*[0 0 1]';
        V(1:3,i_frame) = V_prior(1:3) + cross(V_prior(4:6),W_i_1_r_i);
        % update frame acceleration
        VD(4:6,i_frame) = VD_prior(4:6) + qDD_i*W_R_i*[0 0 1]' + qD_i*cross(V_prior(4:6),W_R_i*[0 0 1]');
        VD(1:3,i_frame) = VD_prior(1:3) + cross(VD_prior(4:6),W_i_1_r_i)+cross(V_prior(4:6),cross(V_prior(4:6),W_i_1_r_i));
        % calculate acceleration with respect to the center of mass
        VD_c(4:6,i_frame) = VD(4:6,i_frame);
        VD_c(1:3,i_frame) = VD(1:3,i_frame) + cross(VD(4:6,i_frame),W_i_r_c) + cross(V(4:6,i_frame),cross(V(4:6,i_frame),W_i_r_c));

        i_frame = i_frame+1;
    end
    i_frame_finger_end(finger_i) = i_frame-1;
end
assert(i_frame == n_frame+1,'[invdyn_ne_T] test condition: wrong n_frame');

%% backward recursion
for i = n_frame:-1:1 % n_q+1:-1:1
    % if the frame is the endeffector of one finger
    [is_ee,index] = ismember(i,i_frame_finger_end);
    if is_ee
        F(:,i) = -F_ext(:,index); % end effector
        continue
    end

    W_T_i = T(:,:,i);
    W_R_i = W_T_i(1:3,1:3);
    % dynamic parameters
    m_i = Mass_n_frame(i);
    I_i = inertia_tensor2matrix(I_n_frame(:,i));
    W_I_i = W_R_i*I_i*W_R_i';
    if i == i_frame_base_end(end)
        % the ee of the lase base
        for j = 1:n_f
            % add force of each finger base
            W_T_ip1 = T(:,:,i_frame_finger_start(j));
            i_T_ip1 =  W_T_i\W_T_ip1;
            W_i_r_ip1 = W_R_i*i_T_ip1(1:3,4);
            % i to center of mass
            W_i_r_c = W_R_i*CoM_n_frame(:,i);
            W_c_r_ip1 = W_i_r_ip1-W_i_r_c;

            F(1:3,i) = F(1:3,i) + F(1:3,i_frame_finger_start(j));
            F(4:6,i) = F(4:6,i) + F(4:6,i_frame_finger_start(j)) - cross(W_c_r_ip1,-F(1:3,i_frame_finger_start(j)));
        end
        F(1:3,i) = F(1:3,i) - m_i*g + m_i*VD_c(1:3,i);
        F(4:6,i) = F(4:6,i) - cross(-W_i_r_c,F(1:3,i))  ...
                + W_I_i*VD_c(4:6,i) + cross(V(4:6,i),W_I_i*V(4:6,i));
    else
        % forces exerted on the current frame by the last frame
        % calculate the W_c_r_ip1
        W_T_ip1 = T(:,:,i+1);
        i_T_ip1 =  W_T_i\W_T_ip1;
        W_i_r_ip1 = W_R_i*i_T_ip1(1:3,4);
        % i to center of mass
        W_i_r_c = W_R_i*CoM_n_frame(:,i);
        W_c_r_ip1 = W_i_r_ip1-W_i_r_c;

        F(1:3,i) = F(1:3,i+1) - m_i*g + m_i*VD_c(1:3,i);
        F(4:6,i) = F(4:6,i+1) - cross(-W_i_r_c,F(1:3,i)) - cross(W_c_r_ip1,-F(1:3,i+1)) ...
            + W_I_i*VD_c(4:6,i) + cross(V(4:6,i),W_I_i*V(4:6,i));
    end
end


% frame force/moment to each joint torque
for i = 1:length(i_frame_q)
    % frame qi to World
    W_T_i = T(:,:,i_frame_q(i));
    W_R_i = W_T_i(1:3,1:3);
    Tau(i) = F(4:6,i_frame_q(i))'*W_R_i*[0 0 1]';
end

end