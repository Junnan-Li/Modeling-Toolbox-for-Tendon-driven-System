% Geometric Jacobian of the end-effectors of one finger 
% 
% input:
%           finger_index: 
%           q: 
% 
% output:
%           J: [6xobj.nja] geometric jacobian
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% Note: the same as J = Jacobian_geom_mdh(mdh,q)
% 
% Junnan Li, junnan.li@tum.de, 01.2023

function J = Jacobian_geom_w_one_finger(obj,finger_index,q)

if nargin == 1
    q = obj.q;
end
assert(length(q)== obj.nj, '[Jacobian_geom_w_one_finger]: dimension of joint vector is incorrect!')
assert(finger_index <= obj.nf, '[Jacobian_geom_w_one_finger]: dimension of joint vector is incorrect!')

q_init = obj.q;
q = reshape(q,[obj.nj,1]);
obj.update_hand(q);


J = zeros(6,obj.nj);

% w_T_fingeri_end
w_T_fi_end = obj.list_fingers(finger_index).get_w_T_ee_inhand;

J_base = zeros(6,obj.njb);
index_base = 1;
if obj.nb ~= 0
    for i = 1:obj.nb
        base_i = obj.base(i);
        for j = 1:base_i.nj
            w_T_b_basei = base_i.get_w_T_base_inhand();
            mdh_i = mdh_struct_to_matrix(base_i.mdh,1);
            base_T_qi = T_mdh_multi(mdh_i(1:j,:));
            w_T_qi = w_T_b_basei * base_T_qi;
            qi_T_fi_end = w_T_qi\w_T_fi_end;
            qi_p_end = qi_T_fi_end(1:3,4);         
            w_R_qi = w_T_qi(1:3,1:3);
            J_qi_p = cross(w_R_qi*[0;0;1],w_R_qi*qi_p_end);
            J_qi_w = w_R_qi*[0;0;1];
            J_base(:,index_base) = [J_qi_p;J_qi_w];
            index_base = index_base + 1;
        end
    end
end
% % w_T_base_end
% w_T_base_end = obj.base(end).get_w_T_ee_inhand;

J_finger = zeros(6,obj.njf);
index_f = 1;
if obj.nf ~= 0
    for i = 1:obj.nf
        finger_i = obj.list_fingers(i);
        for j = 1:finger_i.nj
            if i ~= finger_index
                J_finger(:,index_f) = zeros(6,1);
                index_f = index_f+1;
                continue;
            end 
            w_T_base_fi = finger_i.get_w_T_base_inhand();
            mdh_i = mdh_struct_to_matrix(finger_i.mdh,1);
            base_T_qi = T_mdh_multi(mdh_i(1:j,:));
            w_T_qi = w_T_base_fi * base_T_qi;
            qi_T_fi_end = w_T_qi\w_T_fi_end;
            qi_p_end = qi_T_fi_end(1:3,4);         
            w_R_qi = w_T_qi(1:3,1:3);
            J_qi_p = cross(w_R_qi*[0;0;1],w_R_qi*qi_p_end);
            J_qi_w = w_R_qi*[0;0;1];
            J_finger(:,index_f) = [J_qi_p;J_qi_w];
            index_f = index_f + 1;
        end
    end
end

J = [J_base,J_finger];

if index_base ~= obj.njb+1 || index_f ~= obj.njf+1
    error('index are wrong')
end
obj.update_hand(q_init);
end