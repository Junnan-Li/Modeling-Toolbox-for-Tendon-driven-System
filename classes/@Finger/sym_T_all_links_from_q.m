% generate the symbolic expression of transformation matrices of all links
% with q as input.
% 
% 


function w_T_all_sym = sym_T_all_links_from_q(obj, varargin)

if nargin == 1
    save_res = 1;
else
    save_res = varargin{1};
end

% get all transformation matrix of all links
w_T_b = sym(obj.w_T_base);
w_T_all_sym = sym(zeros(4,4,obj.nl+2)); % base, link1, ... linkn, ee
w_T_all_sym(:,:,1) = w_T_b;
q_sym = sym('q',[obj.nj,1], 'real');
mdh = obj.mdh_ori;
mdh_sym =  sym(mdh_struct_to_matrix(mdh, 1));
mdh_sym(:,3) = mdh_sym(:,3) +  sym([q_sym;0]);
% update all link properties
for i = 1:obj.nl+1
    b_T_i = simplify(T_mdh_multi(mdh_sym(1:i,:)));
    w_T_all_sym(:,:,i+1) = simplify(w_T_b*b_T_i);
end

var_name = {q_sym};

if save_res
    func_name = strcat('output/T_all_links_from_q_', obj.name);
    if ~exist('./output', 'dir')
       mkdir('./output');
    end
    matlabFunction(w_T_all_sym,"File",func_name,...
        "Vars", var_name,"Outputs", {'w_T_link_all'}); %

    func_name = strcat('output/T_all_links_from_q_opt_', obj.name);
    matlabFunction(w_T_all_sym,"File",func_name,...
        "Vars", var_name,"Outputs", {'w_T_link_all'},"Optimize",1); % 
end
end