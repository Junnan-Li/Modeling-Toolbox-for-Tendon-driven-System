

function [sigma_min,sigma_vol,sigma_isotropic] = metric_algebraic_analysis(Matrix)
%   algebraic of the matrix using singular value decomposition
%   Matrix should be a fat matrix which has full row rank

[~,S,~] = svd(Matrix);
num_row = size(Matrix,1);
% num_col = size(Matrix,2);
sigma = zeros(num_row,1);
for i = 1:num_row
    sigma(i) = S(i,i);
end
sigma_min = sigma(end);
sigma_vol = prod(sigma);
sigma_isotropic = sigma_min/sigma(1);


fprintf('Algebraic analysis: Minimum Singular Value: %.6e \n',sigma_min )
fprintf('Algebraic analysis: Volume of the ellipsoid: %.6e \n',sigma_vol )
fprintf('Algebraic analysis: Isotropic Value: %.6e \n',sigma_isotropic )

end