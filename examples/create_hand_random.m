% create a hand object with random mdh, base configuration, and dynamic parameters
% 
% input:
%           hand_name:        name of the finger object
%           varargin:           
%               num_base
%               dof_base
%               num_finger
%               dof_finger
% 
% for example 
%   create_hand_random("hand_example", [2,3,4,3] ): create a hand with 2
%   bases, each of which has 3 DoFs, and 4 fingers, each of which has 3
%   DoFs
% 
% output:   
%           hand_obj:        
% 
function hand_obj = create_hand_random(hand_name, varargin )

if nargin == 1
    num_base = 2;
    dof_base = 2;
    num_finger = 2;
    dof_finger = 3;
else
    num_base = varargin{1}(1);
    dof_base = varargin{1}(2);
    num_finger = varargin{1}(3);
    dof_finger = varargin{1}(4);
end

hand_obj = Hand(hand_name);
for i = 1:num_base
    base_name_i = strcat('base_',num2str(i));
    base_i = create_finger_random(base_name_i, dof_base);
    hand_obj.add_base(base_i);
end
for i = 1:num_finger
    finger_name_i = strcat('finger_',num2str(i));
    finger_i = create_finger_random(finger_name_i, dof_finger);
    hand_obj.add_finger(finger_i);
end
q_hand = rand(hand_obj.nj,1);
hand_obj.update_hand(q_hand);

hand_obj.update_list_viapoints;
hand_obj.update_hand_par_dyn;
hand_obj.update_sim_par;

end