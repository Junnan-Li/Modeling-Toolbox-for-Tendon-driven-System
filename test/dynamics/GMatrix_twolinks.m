function GMatrix = GMatrix_twolinks(m1,m2,l1,l2,la1,la2, q,g)

q1 = q(1);
q2 = q(2);
g1 = m1*g*la1*cos(q1)+m2*g*(l1*cos(q1)+la2*cos(q1+q2));
g2 = m2*g*la2*cos(q1+q2);

GMatrix = [g1;g2];
end