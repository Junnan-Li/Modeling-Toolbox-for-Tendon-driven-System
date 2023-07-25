function handone_sfunction(block)
% rl_control.m      e.anderlini@ucl.ac.uk     23/10/2017
    setup(block);
end

%% Set up the block:
function setup(block)
%   % Register number of dialog parameters:   
%   block.NumDialogPrms = 3;

    % Register number of input and output ports:
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;

    % Set up functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Set up the input ports:
    block.InputPort(1).Dimensions        = 8;        
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(2).Dimensions        = 4;        
    block.InputPort(2).DirectFeedthrough = true;
    % Set up the output port:
    block.OutputPort(1).Dimensions       = 8;        

    % Set block sample time to continuous:
    block.SampleTimes = [0 0];

%     % Setup Dwork:
%     block.NumContStates = 1;

    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
%     block.RegBlockMethod('InitializeConditions',    @InitConditions);  
    block.RegBlockMethod('Outputs',                 @Output);  
%     block.RegBlockMethod('Derivatives',             @Derivative);  
end

% %% Initial conditions:
% function InitConditions(block)
%     % Initialize Dwork:
%     block.ContStates.Data = block.DialogPrm(3).Data;
% end

%% Set up the output:
function Output(block)
    
    q = block.InputPort(1).Data(1:4);
    qD = block.InputPort(1).Data(5:8);
    Tau = block.InputPort(2).Data;
    global finger_handone;
    qDD = finger_handone.fordyn_ne_w_end(q, qD, Tau, zeros(6,6), 1);
    
    block.OutputPort(1).Data = [qD;qDD];
end