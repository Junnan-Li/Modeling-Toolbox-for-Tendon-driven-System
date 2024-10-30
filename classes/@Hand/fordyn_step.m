% Forward dynamic simulation for one step 
% not finished 
% 
% TODO: 
%       1. input of function should be muscle activation
%       2. length of muscle schould be updated 
%       3. tau should be calculated from muscle models
%       4. 
%   
% input:
%           step: 
%           tau: 
% 
% output:
%           state_new:
% 
% 

function state_new = fordyn_step(obj, step, tau)
%  qD, Tau, F_ext, 
state = obj.get_state;
q_state = state(1:obj.nj);
qd_state = state(obj.nj+1:2*obj.nj);

qDD = obj.fordyn_ne_hand_w_end(q_state,qd_state,tau);
 
q_state_new = q_state + qd_state * step;
qd_state_new = qd_state + qDD * step;

obj.update_hand(q_state_new);
l_mus_new = obj.get_muscle_length_all;
alpha_mus_new = zeros(obj.nmus,1); % not updating muscle exitation

state_new = [q_state_new;qd_state_new;l_mus_new;alpha_mus_new];
obj.set_state(state_new);


end