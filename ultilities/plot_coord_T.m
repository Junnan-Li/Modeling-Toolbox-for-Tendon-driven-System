% plot Coordinate from T stack input
% 
% Input :
%       T       [4*n,4]
%       varargin 
% 
% 


function plot_coord_T(T, varargin)

num_T = size(T,1)/4;

if nargin == 1
    plot_par = plot_parameter_init();
elseif nargin == 2
    plot_par = varargin{1};
else
    error('[plot_coord_T]: too much input!!')
end

Color_axis = {'r','b','g'};
for i = 1:num_T
    T_i = T(4*i-3:4*i,:);
    origin_i = T_i(1:3,4);
    scalar = plot_par.axis_len;

    for j = 1:3
        h = quiver3(origin_i(1),origin_i(2),origin_i(3),...
            scalar*T_i(1,j),scalar*T_i(2,j),scalar*T_i(3,j),...
            'Color',Color_axis{j},'LineWidth',plot_par.linewidth);
        set(h,'AutoScale','on', 'AutoScaleFactor',1);
        hold on
    end
    
end
xlabel('x')
ylabel('y')
zlabel('z')
end