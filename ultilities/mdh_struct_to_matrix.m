% transform mdh parameters from structure form to matrix form 
% 
% Input:
%           order: 
%               1: alpha,a,theta,d
%               2: a,alpha,d,theta
%           q: joint configuration
% 
% Output:
%           mdh_matrix: [onj.nja x 4]
% 



function mdh_matrix = mdh_struct_to_matrix(mdh_struct, order)
%#codegen
% assert(isequal(class(D), 'Finger'), 'Input class is not Finger!')

mdh_matrix = [];
len = length(mdh_struct.a);

switch order
    case 1
        for i = 1:len
            
            mdh_matrix(i,1) = mdh_struct.alpha(i);
            mdh_matrix(i,2) = mdh_struct.a(i);
            mdh_matrix(i,3) = mdh_struct.theta(i);
            mdh_matrix(i,4) = mdh_struct.d(i);
        end
        
    case 2
        for i = 1:len
            mdh_matrix(i,1) = mdh_struct.a(i);
            mdh_matrix(i,2) = mdh_struct.alpha(i);
            mdh_matrix(i,3) = mdh_struct.d(i);
            mdh_matrix(i,4) = mdh_struct.theta(i);
        end
    otherwise
        error('func "mdh_struct_to_matrix" error: variable order not defined! \n')
end
end



