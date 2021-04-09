function [coeffsValues, angle0] = interpCoeffs(t, alpha, M, beta, alt, c, alphaTot, settings)
%{
interpCoeffs - interpolation of aerodynamic coefficients.

INPUTS:
        - t, double [1,1], integration time, [s];
        - alpha, double[1,1], angle of attack, [];
        - M, double[1,1], mach number, [];
        - beta, double[1,1], sideslip angle, [];
        - alt, double[1,1], altitude, [m];
        - c, double[1,1], aerobrakes control variable, [];
        - alphaTot, double[1,1], total angle of attack, [];
        - settings, struct, rocket and simulation data.

OUTPUTS:
        - coeffsValues, array [16,1],  aerodynamic coefficients;
        - angle0, array [2,1], array of the reference aerodynamic angles.

CALLED FUNCTIONS: -

VERSIONS:
-
%}

%% Load data:
CoeffsE = settings.CoeffsE;  % Empty Rocket Coefficients
CoeffsF = settings.CoeffsF;  % Full Rocket Coefficients

tb = settings.tb;

A_datcom = settings.Alphas*pi/180;
B_datcom = settings.Betas*pi/180;
H_datcom = settings.Altitudes;
M_datcom = settings.Machs;

%% Interpolation:
% Last two entries of cellT and inst are for the evaluation of the XCP for
% alpha = alphaTot and beta = 0.
cellT = {A_datcom, M_datcom, B_datcom, H_datcom, A_datcom, B_datcom};
inst = [alpha, M, beta, alt, alphaTot, 0];

index = zeros(6, 1);
for i = 1:6
    [~, index(i)] = min(abs(cellT{i} - inst(i)));
end

coeffsNames = {'CA','CYB','CY','CNA','CN','CLL','CLLP','CMA','CM',...
    'CMAD','CMQ','CLNB','CLN','CLNR','CLNP','X_C_P'};
coeffsValues = nan(16, 1);

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
