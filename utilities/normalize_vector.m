








function normalized_vector = normalize_vector(matrix, dimension)


acc_r_max = max(matrix,[],dimension);
acc_r_min = min(matrix,[],dimension);
normalized_vector = (matrix - acc_r_min) ./ (acc_r_max-acc_r_min);


end