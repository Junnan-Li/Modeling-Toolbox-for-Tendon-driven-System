function dqdt = fordyn(obj,t0 ,state, Tau_i, F_ext,mex)

q = state(1:obj.nj);
qd = state(obj.nj+1:end);
qdd = obj.fordyn_ne_w_end(q, qd, Tau_i, F_ext ,mex);
dqdt = [qd;qdd];

end