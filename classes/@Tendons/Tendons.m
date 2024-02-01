%% Class of Tendons:
% 
%       Create a Tendons:


% Comment: 
%       1. property 'j_index' make the tendon affect all the joints before
%       i-th joint. 


classdef Tendons < handle
    
    properties (Access = public)
        name                 % [char] name of tendon
%         j_index              % [1] index of the last joint that tendon pulls 
%         routing              % [njx1] flexion/ (+1) or extension (-1
        f_limits             % [1x2] [min max] tendon forces
        
        stiffness            % [1] stiffness
        damping              % [1] damping
        

    end
    properties (SetAccess = private)
        nj_finger            % [1] number of joints from finger.nj 
        index                % [1] index of the tendon of a finger
        par_MA_poly3         % [4xnj_finger] parameters of the moment arm w.r.t. q. MA =  ax3 + bx2+ cx + d
        ma_value             % [nj_fingerx1] moment arms in m, + for flexion 
        ma_limits            %[4xnj_fingerx2] minimal and maximal abs value of tendon
    end
    
    methods
        function obj = Tendons(name)
            %TENDON initialize
            obj.name = name;
            obj.nj_finger = 0;
            obj.ma_value = [];
%             obj.j_index = length(find(routing)); % the tendon will affect the first j_index joints
            obj.stiffness = 0; %
            obj.damping = 0; %
            obj.index = 0; %
            obj.f_limits = [0.5, 100];
            % init poly3 parameters
            obj.par_MA_poly3 = [];
            obj.ma_limits = [];
%             for i = 1:obj.j_index
%                 obj.par_MA_poly3(:,i) = [0;0;0;0.01]; % default value 10mm
%             end

        end
        
        function init_tendon_par(obj, nj_finger, index )
            % update parameters from finger class
            obj.nj_finger = nj_finger;
            obj.par_MA_poly3 = zeros(4,obj.nj_finger);
            obj.ma_limits = zeros(obj.nj_finger,2);
            obj.index = index;
            obj.ma_value = zeros(obj.nj_finger,1); 
            obj.ma_limits = repmat([-0.03,0.03],obj.nj_finger,1); % init ma limit 5~20mm
        end
        function update_ma_limits(obj,j_index,ma_limits_i) 
            obj.ma_limits(j_index,:) = ma_limits_i;
        end
        
        function update_momentarm(obj, q)
            % update moment arm 
            obj.ma_value = obj.cal_ma_poly3(q);
        end
        
        function set_par_MA_poly3(obj, poly3_par, q)
            % change the parameters of the poly3 MA function regarding the
            % j_index-th joint and update the moment arm value
            % 
            
            assert(all(size(poly3_par) == [4,obj.nj_finger])==1, 'dimension of par_MA_poly3 is incorrect!')
            obj.par_MA_poly3 = poly3_par;
            obj.update_momentarm(q);
        end
        
        function ma_value = cal_ma_poly3(obj, q)
            % calculate the moment arm value with q using poly3 expression
            % MA =  ax3 + bx2+ cx + d
            ma_value = zeros(obj.nj_finger,1);
            for i = 1:obj.nj_finger
                q_i = q(i);
                a = obj.par_MA_poly3(1,i);
                b = obj.par_MA_poly3(2,i);
                c = obj.par_MA_poly3(3,i);
                d = obj.par_MA_poly3(4,i);
                moment_arm_abs_i = a*q_i^3 + b*q_i^2 + c*q_i + d; 
                ma_limit_i_abs = obj.ma_limits(i,:);
                ma_value(i) = max(ma_limit_i_abs(1), min(moment_arm_abs_i, ma_limit_i_abs(2)));
                
            end
            
            
        end
        

    end
end

