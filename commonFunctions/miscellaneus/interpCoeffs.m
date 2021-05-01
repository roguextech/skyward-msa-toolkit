function [coeffsValues, angle0] = interpCoeffs(t, alpha, M, beta, alt, c, settings)
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

%% Load data
CoeffsE = settings.CoeffsE;  % Empty Rocket Coefficients
CoeffsF = settings.CoeffsF;  % Full Rocket Coefficients

tb = settings.tb;

datcomAlphas = settings.Alphas*pi/180;
datcomBetas = settings.Betas*pi/180;
datcomAlts = settings.Altitudes;
datcomMachs = settings.Machs;

%% Interpolation
cellT = {datcomAlphas, datcomMachs, datcomBetas, datcomAlts};
inst = [alpha, M, beta, alt]; 

index = zeros(4, 1);
for i = 1:4
    [~, index(i)] = min(abs(cellT{i} - inst(i)));
end

coeffsNames = {'CA','CYB','CY','CNA','CN','CLL','CLLP','CMA','CM',...
    'CMAD','CMQ','CLNB','CLN','CLNR','CLNP'}; %,'X_C_P'};
coeffsValues = nan(15, 1);

for i = 1:15
    
    CmatE = CoeffsE.(coeffsNames{i});
    CmatF = CoeffsF.(coeffsNames{i});
    
    VE = CmatE(index(1), index(2), index(3), index(4), c);

    if t <= tb
        VF = CmatF(index(1), index(2), index(3), index(4));
        
        coeffsValues(i) =  t/tb*(VE - VF) + VF;
    else 
        coeffsValues(i) = VE;
    end

end

angle0 = [datcomAlphas(index(1)); datcomBetas(index(3))];

end
