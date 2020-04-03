
% TESTED 
clear all
oneStep = load('EER_oneStep');  oneStep = oneStep.oneStep;
crt_cueLev = 2; id = 3;
if crt_cueLev == 3 % no more cues to reveal 
    EER = 0;
else
    id_crt =  ceil(id/2^(3-crt_cueLev));
    EER = oneStep{crt_cueLev+1}(id_crt);
end
disp(EER)