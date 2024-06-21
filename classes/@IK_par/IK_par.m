%% Class of Joints:
% 
%       Create a Link:



classdef IK_par < handle
    %JOINTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ikpar_NR
        ikpar_LM
    end
    properties (SetAccess = private)

    end
    methods
        function obj = IK_par()
            % IK_par Construct an instance of this class
            
            % parameter for Newton Raphson method
            obj.ikpar_NR = struct();
            obj.ikpar_NR.iter_max = 100;
            obj.ikpar_NR.retry_num = 10;
            obj.ikpar_NR.tol = [1e-6*ones(3,1);1e-5*ones(3,1)];
            obj.ikpar_NR.alpha = 0.15;
            obj.ikpar_NR.visual = 0;

            % parameter for levenberg-marquardt method

            obj.ikpar_LM = struct();
            obj.ikpar_LM.iter_max = 100;
            obj.ikpar_LM.retry_num = 10;
            obj.ikpar_LM.tol = [1e-6*ones(3,1);1e-5*ones(3,1)];
            obj.ikpar_LM.W_e = diag([1,1,1,1,1,1]);
            obj.ikpar_LM.W_d = 1e-6;
            obj.ikpar_LM.visual = 0;
        end
        

    end
end