




% code generation for invdyn_ne_mdh.m with four joints


% n_q = 4;
% input_dimension = {zeros(n_q,1),zeros(n_q,1),zeros(n_q,1),zeros(n_q+1,4),...
%     zeros(n_q+1,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(3,n_q+1),...
%     zeros(6,n_q+1),zeros(3,1)};

codegen invdyn_ne_mdh.m -args {zeros(4,1),zeros(4,1),zeros(4,1),zeros(5,4),zeros(5,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(3,5),zeros(6,5),zeros(3,1)}