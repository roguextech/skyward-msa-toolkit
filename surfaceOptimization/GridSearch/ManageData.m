function [settings] = ManageData (data, settings)
%{
ManageData - This function takes the values stored in the data struct, to
store them back in the settings one.

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 18/10/2019

%}

% Coefficients in full configuration
settings.CoeffsF = data.full.Coeffs;

% Coefficients in empty configuration
settings.CoeffsE = data.empty.Coeffs;

s = data.full.State;
settings.Alphas = s.Alphas';
settings.Betas = s.Betas';
settings.Altitudes = s.Altitudes';
settings.Machs = s.Machs';

end