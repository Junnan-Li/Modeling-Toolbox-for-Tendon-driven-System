

% clc
% clear all
% echotcpip("on",8051)
% t = tcpip('localhost',55000,'NetworkRole','Server');
% tcpipServer
% t = tcpserver("127.0.0.1",8051);
u = udpport("IPV4")
while(1)
    data_1 = ones(1,12);
    data_2 = zeros(1,26);
    data_3 = zeros(1,42);
    data_4 = zeros(1,1);
    data = [data_1,data_2,data_3,data_4];
%     write(t,data,'double');
    write(u,data,"double","127.0.0.1",8051);
end

% echotcpip("off")
