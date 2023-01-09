function Mass_matrix = MassMatrix_twolinks(m1,m2,Izz1,Izz2,l1,l2,la1,la2, q)

q1 = q(1);
q2 = q(2);
m11 = m1*la1^2+Izz1+m2*(l1^2+la2^2+2*l1*la2*cos(q2))+Izz2;

m12 = m2*(la2^2+l1*la2*cos(q2))+Izz2;
m21 = m12;

m22 = m2*la2^2+Izz2;


Mass_matrix = [m11,m12;m21,m22];
end