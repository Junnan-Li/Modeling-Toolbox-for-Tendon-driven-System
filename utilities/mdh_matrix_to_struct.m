% transform mdh parameters from matrix form to struct form  
% Input:
%           order: 
%               1: alpha,a,theta,d
%               2: a,alpha,d,theta
%           q: joint configuration
% 
% Output:
%           mdh_matrix: [onj.nja x 4]
% 


function mdh_struct = mdh_matrix_to_struct(mdh_matrix, order)
%
% assert(isequal(class(D), 'Finger'), 'Input class is not Finger!')

mdh_struct = struct();
% len = length(mdh_struct.a);

switch order
    case 1
        mdh_struct.alpha = mdh_matrix(:,1);
        mdh_struct.a = mdh_matrix(:,2);
        mdh_struct.theta = mdh_matrix(:,3);
        mdh_struct.d = mdh_matrix(:,4);
        
    case 2
        mdh_struct.a = mdh_matrix(:,1);
        mdh_struct.alpha = mdh_matrix(:,2);
        mdh_struct.d = mdh_matrix(:,3);
        mdh_struct.theta = mdh_matrix(:,4);
        
    otherwise
        error('func "mdh_matrix_to_struct" error: variable order not defined! \n')
end
end