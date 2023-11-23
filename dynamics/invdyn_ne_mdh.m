% recursive Newton-Euler method to compute inverse dynamic with mdh
% only end-effector external force input
% parameters
% Input:
%     q,qD,qDD:         [n_qx1] joint states
%     mdh:              [n_q+1x4] in sequence of [alpha,a,theta,d] To be
%                       noted that the theta should exclude from mdh parameters 
%     Mass:             [n_q+1] mass vector. the first mass is the base 
%     X_base:           [6x1] eulerxyz
%     XD_base:          [6x1] translational and angular velocity
%     F_ext:            [6x1] force/moment that exerted by the environment to the endeffector frame. 
%     CoM:              [3xn_q+1] in local frame; last column is endeffector
%     I:                [6xn_q+1] in local frame with respect to center of mass. in sequence of [xx yy zz yz xz xy]
%     g:                [3x1] gravity term in world frame
% 
% Output:
%     Tau:              [n_qx1] joint torque 
%     F:                [6xn_q+2] force/moment vectors that the i link exert to the i+1 link
%     W_T_allframe:     [4x4xn_q+2] Homogeneous transformation from frame i to world frame. 1: base, 2~n_q+1: link, end:  

% internal variable:
%   n_q: number of joints
%   n_q: number of frames (n_q+2)
%   V: [6xn_q+2] linear & angular velocity of each frames
%   VD: [6xn_q+2] linear & angular accelerations of each frames
%   VD_c: [6xn_q+2] linear & angular accelerations of each center of mass;
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


function [Tau,F,W_T_allframe] = invdyn_ne_mdh(q,qD,qDD,mdh, Mass, X_base, XD_base,XDD_base, F_ext, CoM, I, g)
%#codegen

% test input dimension
n_q = length(q);
n_f = n_q+3;
assert(n_q == length(qD)&&n_q == length(qDD),'dimension error!');

% F (force and moment) with respect to the Wolrd Frame
% dimension: [6xn_q+2]; 1st column is base, last column is endeffector(input)
Tau = zeros(n_q,1);
F = zeros(6,n_q+2);
F(:,end) = -F_ext;

% V with respect to the Wolrd Frame 
% dimension: [6xn_q+1]; 1st column is base, last column is endeffector
V = zeros(6,n_q+2);
VD = zeros(6,n_q+2);
V(:,1) = XD_base;
VD(:,1) = XDD_base;
VD_c = VD;
% 
W_T_allframe = zeros(4,4,n_f-1);
mdh(1:n_q,3) = mdh(1:n_q,3)+q; % update mdh parameters with the current q
% homogeneous transformation from Base to World frame
W_T_allframe(:,:,1) = [euler2R_XYZ(X_base(4:6)),X_base(1:3);...
                0 0 0 1];
for i = 2:n_f-1
    W_T_allframe(:,:,i) = W_T_allframe(:,:,1)*T_mdh_multi(mdh(1:i-1,:));
end
% update VD_c of base
W_b_r_c = euler2R_XYZ(X_base(4:6))*CoM(:,1);
VD_c(1:3,1) = VD(1:3,1) + cross(VD(4:6,1),W_b_r_c) + cross(V(4:6,1),cross(V(4:6,1),W_b_r_c));


% forward recursion: 
for i = 2:n_q+2
    % i: index of frame;
    if i == n_q+2 % endeffector
%         q_i = 0;
        qD_i = 0;
        qDD_i = 0;
        CoM_i = [0;0;0]; % virtual endeffector link 
    else
%         q_i = q(i-1);
        qD_i = qD(i-1);
        qDD_i = qDD(i-1);
        CoM_i = CoM(:,i);
    end
    % frame qi to World
    W_T_i = W_T_allframe(:,:,i);
    W_R_i = W_T_i(1:3,1:3);
    W_T_i_1 = W_T_allframe(:,:,i-1);
    i_1_T_i =  W_T_i_1\W_T_i;
    
    W_i_1_r_i = W_T_i_1(1:3,1:3)*i_1_T_i(1:3,4);% frame i-1 to frame i
    W_i_r_c = W_R_i*CoM_i;% frame i to center of mass
    
    % update frame velocity 
    V(4:6,i) = V(4:6,i-1) + qD_i*W_R_i*[0 0 1]';
    V(1:3,i) = V(1:3,i-1) + cross(V(4:6,i-1),W_i_1_r_i);
    % update frame acceleration 
    VD(4:6,i) = VD(4:6,i-1) + qDD_i*W_R_i*[0 0 1]' + qD_i*cross(V(4:6,i-1),W_R_i*[0 0 1]');
    VD(1:3,i) = VD(1:3,i-1) + cross(VD(4:6,i-1),W_i_1_r_i)+cross(V(4:6,i-1),cross(V(4:6,i-1),W_i_1_r_i));
    % calculate acceleration with respect to the center of mass
    VD_c(4:6,i) = VD(4:6,i);
    VD_c(1:3,i) = VD(1:3,i) + cross(VD(4:6,i),W_i_r_c) + cross(V(4:6,i),cross(V(4:6,i),W_i_r_c));
 end

% backward recursion
for i = n_q+1:-1:1
    % start from the last joint frame (endeffector force/moment is given)
    % frame qi to World
    W_T_i = W_T_allframe(:,:,i);
    W_R_i = W_T_i(1:3,1:3);
    W_T_ip1 = W_T_allframe(:,:,i+1);
    i_T_ip1 =  W_T_i\W_T_ip1;
    W_i_r_ip1 = W_R_i*i_T_ip1(1:3,4);
    % i to center of mass
    
    W_i_r_c = W_R_i*CoM(:,i);
    W_c_r_ip1 = W_i_r_ip1-W_i_r_c;
    
    % dynamic parameters
    m_i = Mass(i);
    I_i = inertia_tensor2matrix(I(:,i));
    W_I_i = W_R_i*I_i*W_R_i';
    
    % forces exerted on the current frame by the last frame
    F(1:3,i) = F(1:3,i+1) - m_i*g + m_i*VD_c(1:3,i);
    F(4:6,i) = F(4:6,i+1) - cross(-W_i_r_c,F(1:3,i)) - cross(W_c_r_ip1,-F(1:3,i+1)) ...
        + W_I_i*VD_c(4:6,i) + cross(V(4:6,i),W_I_i*V(4:6,i));

end


% frame force/moment to joint torque
for i = 2:n_q+1
    % frame qi to World
    W_T_i = W_T_allframe(:,:,i);
    W_R_i = W_T_i(1:3,1:3);
    Tau(i-1) = F(4:6,i)'*W_R_i*[0 0 1]';
end

end