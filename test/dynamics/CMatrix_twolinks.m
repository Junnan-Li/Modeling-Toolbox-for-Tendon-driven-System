function C_matrix = CMatrix_twolinks(m1,m2,l1,l2,la1,la2, q,q_dot)

q1 = q(1);
q2 = q(2);
q1_dot = q_dot(1);
q2_dot = q_dot(2);
c11 = -m2*l1*la2*sin(q2)*q2_dot;

c12 = -m2*l1*la2*sin(q2)*(q1_dot+q2_dot);
c21 = m2*l1*la2*sin(q2)*q1_dot;

c22 = 0;


C_matrix = [c11,c12;c21,c22];
end