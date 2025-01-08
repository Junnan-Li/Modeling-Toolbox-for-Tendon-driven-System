function [tau] = double_exp_Deshpande(A,B,C,D,E,F,G,H,q_f)
% Double exponential function for representing the joint viscoelastic torque 
% 
% Output:
%       tau:    in N-cm
% 
% Reference:
% Deshpande, Ashish D., Nick Gialias, and Yoky Matsuoka. “Contributions of Intrinsic Visco-Elastic Torques 
% during Planar Index Finger and Wrist Movements.” IEEE Transactions on Biomedical Engineering 59, no. 2 
% (2012): 586–94. https://doi.org/10.1109/TBME.2011.2178240.

tau = A .* (exp(-B .* (q_f - E + G))-1 ) - ...
    C .* (exp(D .* (q_f - F + H))-1);

end