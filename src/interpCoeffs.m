function [coeffsValues, angle0] = interpCoeffs(t,alpha,M,beta,alt,c,alpha_tot,settings)
%{
interpCoeffs - interpolation of aerodynamic coefficients.

INPUTS:
            - t, integration time;
            - alpha, angle of attack;
            - M, mach number;
            - beta, sideslip angle;
            - alt, altitude (m.s.l.);
            - c, aerobrakes control variable;
            - alpha_tot, total angle of attack;
            - settings, input structure;

OUTPUTS:
            - coeffsValues, array of aerodynamic coefficients;
            - angle0, array of the reference aerodynamic angles;
%}

%% Load data:
CoeffsE = settings.CoeffsE;  % Empty Rocket Coefficients
CoeffsF = settings.CoeffsF;  % Full Rocket Coefficients

tb = settings.tb;

A_datcom = settings.Alphas*pi/180;
B_datcom = settings.Betas*pi/180;
H_datcom = settings.Altitudes;
M_datcom = settings.Machs;

%% Interpolation at boundaries:
if M > M_datcom(end)
    
    M = M_datcom(end);
    
end

if M < M_datcom(1)
    
    M = M_datcom(1);
    
end

if alpha > A_datcom(end)
    
    alpha = A_datcom(end);
    
elseif alpha < A_datcom(1)
    
    alpha = A_datcom(1);
    
end

if beta > B_datcom(end)
    
    beta = B_datcom(end);
    
elseif beta < B_datcom(1)
    
    beta = B_datcom(1);
end

if alt > H_datcom(end)
    
    alt = H_datcom(end);
    
elseif alt < H_datcom(1)
    
    alt = H_datcom(1);
end

%% Interpolation:

% Last two entries of cellT and inst are for the evaluation of the XCP for
% alpha = alpha_tot and beta = 0.
cellT = {A_datcom, M_datcom, B_datcom, H_datcom, A_datcom, B_datcom};
inst = [alpha, M, beta, alt, alpha_tot, 0];

index = zeros(6,1);
for i = 1:6
    [~, index(i)] = min(abs(cellT{i} - inst(i)));
end

coeffsNames = {'CA','CYB','CY','CNA','CN','CLL','CLLP','CMA','CM',...
    'CMAD','CMQ','CLNB','CLN','CLNR','CLNP','X_C_P'};
coeffsValues = nan(16,1);

for i = 1:16
    
    CmatE = CoeffsE.(coeffsNames{i});
    CmatF = CoeffsF.(coeffsNames{i});
    
    VE = CmatE(index(1), index(2), index(3), index(4), c);
    
    if i == 16, VE = CmatE(index(5), index(2), index(6), index(4), c); end
  
    if t <= tb
        VF = CmatF(index(1), index(2), index(3), index(4));
        
        if i == 16, VE = CmatF(index(5), index(2), index(6), index(4)); end
        
        coeffsValues(i) =  t/tb*(VE-VF)+VF;
    else 
        coeffsValues(i) = VE;
    end

end

angle0 = [A_datcom(index(1)); B_datcom(index(3))];

end
