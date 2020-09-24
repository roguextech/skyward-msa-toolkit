function dY = VertDyn(~, Y, settings)

z = Y(1);
v = Y(2);

m = 12.55;
% Cd = 0.25;
D = 0.16;
S = pi*D^2/4;
g = 9.81;
[~, a, ~, rho] = atmoscoesa(z);
Ma = v/a;

A_datcom = settings.Alphas*pi/180;
B_datcom = settings.Betas*pi/180;
H_datcom = settings.Altitudes;
M_datcom = settings.Machs;
Cd = interp4_easy(A_datcom, M_datcom, B_datcom, H_datcom, settings.CoeffsE.CA, 0, Ma, 0, z);

dY(1) = v;
dY(2) = -0.5*rho*v^2*S*Cd/m - g;

dY = dY';