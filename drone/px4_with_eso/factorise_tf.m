function [factorised_TF] = factorise_tf(TF)
% Removes the same poles and zeros in a transfer function
%   Detailed explanation goes here

% Extract zeros,poles and DC gain from transfer function.
[zeros_tf,poles_tf,K] = zpkdata(TF);
% zeros of transfer function
zeros_tf = cell2mat(zeros_tf);
% poles of transfer functions
poles_tf = cell2mat(poles_tf);
% Comparison accuracy for floating point variables
eps = 1e-4;

zero_fact_loc = [];

for z=1:length(zeros_tf)
    
    zeros_val = zeros_tf(z);
    
    if any(abs(poles_tf-zeros_val) < eps)
        copies = find( abs(poles_tf-zeros_val) < eps,1);
        % factorise poles
        poles_tf(copies(1)) = [];
        % save location of zeros to be factorised out later
        zero_fact_loc = [zero_fact_loc,z];
    end
end
% Factorise zeros out
zeros_tf(zero_fact_loc) = [];

% Get in transfer function form
[num,dem] = zp2tf(zeros_tf,poles_tf,1);
factorised_TF = K*tf(num,dem);
end

