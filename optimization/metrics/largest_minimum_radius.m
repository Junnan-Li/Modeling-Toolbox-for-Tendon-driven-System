% Calculate the largest minimum radius of a polytope centered at the
% specified point using x = lsqlin(C,d,A,b) by means of finding the
% nearest points on the surface of the polytope(L2 norm) to the specified point.
% Input:
%       Vertex: vertex points of the polytope (V-rep)
%       Center: center of the sphere/circle (normally [0,0,0])
%       
% Outout:
%       radius: radius of the largest minimum sphere (o if point is not contained)
%       X: coordinate of the nearest point on the surface
% 
%   Junnan Li, junnan.li@tum.de, MIRMI, 2022

function [radius, X] = largest_minimum_radius(Vertex, Center)

% assert(all(size(Center)== [3,1]), 'input should be a 3 x 1 vector ' )

d = length(Center);
% build the polytope using vertex
P = Polyhedron('V',Vertex);

% test if center is contained in the polytope
contained = P.contains(Center);


% finding the surfaces of the polytope
ls_A = P.A;
ls_b = P.b;
x = []; % nearest point coordinates on each surface
x_r = []; % distance to the center points
options = optimoptions('lsqlin','Display','off');
for i = 1:size( P.A,1)
    % looking for the point on the surface of polytope
    ls_Aeq = P.A(i,:);
    ls_beq = P.b(i,:);
    % objective is the distance to the center point
    ls_C = eye(d);
    ls_d = Center;
    
    x(i,:) = lsqlin(ls_C,ls_d,ls_A,ls_b,ls_Aeq,ls_beq,[],[],[],options);%,ls_Aeq,ls_beq,lb,ub)
    x_r(i) = pdist([x(i,:);Center'],'euclidean'); 
end


[radius,I] = min(x_r);
X = x(I,:);

if ~contained
    X = 0;
end

end






