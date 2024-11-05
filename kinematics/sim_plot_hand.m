% plot hand in simulink 
% 
% input:



function status = sim_plot_hand(w_T_all_links,n_links,q_index)
%#codegen

persistent h
if isempty(h)
    h = figure(1);
    hold on;
    title("hand visualization");
    xlabel("x");
    ylabel("y");
    ylabel("z");
end
% nl = q_index(end,2);
nb = n_links(1);
nf = n_links(2);
nb_link =  q_index(n_links(1),2);
w_p_base = zeros(3,nb_link);
for i = 1:nb_link
    w_p_base(:,i) = w_T_all_links(1:3,4,i);
end
plot3(w_p_base(1,:)', w_p_base(2,:)',w_p_base(3,:)', ".-",'MarkerSize',30,...
    'MarkerFaceColor','r','LineWidth',2,'Color','b');
hold on
% each finger
for i = 1:nf
    nf_link = q_index(nb+i,2)-q_index(nb+i,1)+1;
    w_p_finger = zeros(3,nf_link);
    index_link_start = q_index(nb+i,1)-1;
    for j = 1:nf_link
        index_link = index_link_start+j;
        w_p_finger(:,j) = w_T_all_links(1:3,4,index_link);
    end
    plot3(w_p_finger(1,:)', w_p_finger(2,:)',w_p_finger(3,:)', ".-",'MarkerSize',20,...
    'MarkerEdgeColor','r','LineWidth',2,'Color','b');
    hold on
end
hold off
axis equal
status = 1;
pause(0.1)
end