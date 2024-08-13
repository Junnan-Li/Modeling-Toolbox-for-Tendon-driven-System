% generate the symbolic expression of transformation matrices of all links
% with q as input.
% 
% 


function w_T_all_links = sim_w_T_all_links_from_q(w_T_b,mdh,mdh_index, nb, q)


nl = size(w_T_b,1)/4;

% get all transformation matrix of all links
w_T_b_prior = eye(4);
w_T_all_links_sym = zeros(4,4,obj.nl);
link_index = 1;
if obj.njb ~= 0
    for i = 1:obj.nb
        base_i = obj.base(i);
        q_sym_i = q_sym(obj.index_q_b(i,1):obj.index_q_b(i,2));
        mdh = base_i.mdh_ori;
        mdh_sym =  sym(mdh_struct_to_matrix(mdh, 1));
        mdh_sym(:,3) = mdh_sym(:,3) + sym([q_sym_i;0]);
        w_T_bi = sym(base_i.w_T_base);
        w_T_bi_new = w_T_b_prior*w_T_bi;
        % update all link properties
        for j = 2:base_i.nl
            b_T_i = simplify(T_mdh_multi(mdh_sym(1:j,:)));
            w_T_all_links_sym(:,:,link_index) = simplify(w_T_bi_new*b_T_i);
            link_index = link_index+1;
        end
        % w_T_b_prior is the base transformation for the next finger
        w_T_b_prior = simplify(w_T_bi_new*T_mdh_multi(mdh_sym(1:base_i.nl+1,:)));
    end
end
if obj.njf ~= 0
    for i = 1:obj.nf
        finger_i = obj.list_fingers(i);
        q_sym_i = q_sym(obj.index_q_f(i,1):obj.index_q_f(i,2));
        mdh = finger_i.mdh_ori;
        mdh_sym =  sym(mdh_struct_to_matrix(mdh, 1));
        mdh_sym(:,3) = mdh_sym(:,3) + sym([q_sym_i;0]);
        w_T_bi = sym(finger_i.w_T_base);
        w_T_bi_new = w_T_b_prior*w_T_bi;
        % update all link properties
        for j = 2:finger_i.nl
            b_T_i = simplify(T_mdh_multi(mdh_sym(1:j,:)));
            w_T_all_links_sym(:,:,link_index) = simplify(w_T_bi_new*b_T_i);
            link_index = link_index+1;
        end
    end
end

var_name = {q_sym};

if save_res
    func_name = strcat('output/T_all_links_from_q_', obj.name);
    if ~exist('./output', 'dir')
       mkdir('./output');
    end
    matlabFunction(w_T_all_links_sym,"File",func_name,...
        "Vars", var_name,"Outputs", {'w_T_link_all'}); %

    func_name = strcat('output/T_all_links_from_q_opt_', obj.name);
    matlabFunction(w_T_all_links_sym,"File",func_name,...
        "Vars", var_name,"Outputs", {'w_T_link_all'},"Optimize",1); % 
end
end