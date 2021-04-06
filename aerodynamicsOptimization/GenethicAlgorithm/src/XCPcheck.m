function [c, ceq] = XCPcheck(x, datcom, settings)
%{
optimizationGA - cost function of the optimization

INPUTS:
- x,        double [6, 1], optimization variable, check the config for explanation;
- datcom,   struct (),     variables needed in Datcom
- settings, struct (motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind), 
                   simulation data.
OUTPUTS:
- c,        double [1, 1], non linear constrain of the optimization.

CALLED FUNCTIONS: createFor006, datcomParser, launchPadFreeDyn, windConstGenerator

REVISIONS:
- 0     21/10/20,   release     Adriano Filippo Inno
%}

datcomPath = '../../commonFunctions/Datcom/';

%% RETRIVING DATCOM VARS FROM THE OPTIMIZATION VARIABLES
% x is looping
datcom.Chord1 = x(1)/100;
datcom.Chord2 = x(2)/100;
datcom.Height = x(3)/100;

if x(4) == 1
    datcom.shape = 'iso';
elseif x(4) == 2
    datcom.shape = 'rect';
elseif x(4) == 3
    datcom.shape = 'parall';
end

datcom.Lnose = x(5)/100;

if x(6) == 1
    datcom.OgType = "KARMAN";
elseif x(6) == 2
    datcom.OgType = 'HAACK';
elseif x(6) == 3
    datcom.OgType = 'OGIVE';
elseif x(6) == 4
    datcom.OgType = "POWER";
    datcom.NosePower = 1/3;
elseif x(6) == 5
    datcom.OgType = "POWER";
    datcom.NosePower = 1/2;
elseif x(6) == 6
    datcom.OgType = "POWER";
    datcom.NosePower = 3/4;
end

%% AERODYNAMICS STATES - LAUNCHPAD
% states to compute the exit pad velocity
datcom.Mach = 0.05;
datcom.Alpha = [-0.1, 0, 0.1];
datcom.Beta = 0;
datcom.Alt = settings.z0;

%% AERODYNAMICS COEFFICIENT - LAUNCHPAD
xcg = settings.xcg - settings.Lnose;
datcom.xcg = xcg(1) + datcom.Lnose;
createFor006(datcom, settings, datcomPath);
[Coeffs0, ~] = datcomParser();

%% LAUNCHPAD DYNAMICS
%%% Initial State
X0pad = [0; 0; 0; 0];
%%% Attitude
Q0 = angleToQuat(settings.PHI, settings.OMEGA, 0*pi/180)';

[Tpad, Ypad] = ode113(@launchPadFreeDyn, [0, 10], X0pad, settings.ode.optionspad,...
    settings, Q0, Coeffs0.CA(2));

%% COMPUTING THE LAUNCHPAD STABILITY DATA
% both lateral and longitudinal XCP involved
% the launchpad dynamics is used to write the aerodyanmics states

[~, a, ~, ~] = atmoscoesa(settings.z0);  % sound speed @launchpad
datcom.Mach = Ypad(end, 4)/a;            % Mach @launchpad
datcom.Alt = settings.z0;                % Altitude @launchpad

%%% pre-allocation
XCPlon = zeros(8, 1);
XCPlat = zeros(8, 1);
Az = linspace(0, 360-360/8, 8)*pi/180;   % Wind directions to be looped
for i = 1:8 
settings.wind.Az = Az(i);                % set the direction
    [uw, vw, ww] = windConstGenerator(settings.wind);
    inertialWind = [uw, vw, ww];
    bodyWind = quatrotate(Q0', inertialWind);
    bodyVelocity = [Ypad(end, 4), 0, 0];
    Vr = bodyVelocity - bodyWind;
    ur = Vr(1); vr = Vr(2); wr = Vr(3);
    alphaExit = round(atand(wr/ur), 1);
    betaExit = round(atand(vr/ur), 1);
    
    %%% alpha vector imposed to be symmetric and populated
    if alphaExit ~= 0
        alphaVectorPositive =  [abs(alphaExit) 1 2.5 5 10 15 20];
        alphaVectorPositive = sort(alphaVectorPositive);
        datcom.Alpha = unique(sort([-alphaVectorPositive, 0, alphaVectorPositive]));
    else 
        datcom.Alpha = [-7.5 -5 -2.5 -1 0 1 2.5 5 7.5];
    end
    
    indexAlpha = find(alphaExit == datcom.Alpha);
    datcom.Beta = betaExit;
    
    %%% aerodynmics coefficient - full
    datcom.xcg = xcg(1) + datcom.Lnose;
    createFor006(datcom, settings, datcomPath);
    [CoeffsF, ~] = datcomParser();
    
    %%% aerodynmics coefficient - empty
    datcom.xcg = xcg(2) + datcom.Lnose;
    createFor006(datcom, settings, datcomPath);
    [CoeffsE, ~] = datcomParser();

    %%% longitudnal XCP
    XCPfullLon = -CoeffsF.X_C_P(indexAlpha);
    XCPemptyLon = -CoeffsE.X_C_P(indexAlpha);
    XCPlon(i) = Tpad(end)/settings.tb*(XCPemptyLon - XCPfullLon) + XCPfullLon;
    
    %%% lateral XCP
    if betaExit ~= 0                     % if beta = 0 it's not possible to compute it
        XCPfullLat = -CoeffsF.CLN(indexAlpha)/CoeffsF.CY(indexAlpha);
        XCPemptyLat = -CoeffsE.CLN(indexAlpha)/CoeffsE.CY(indexAlpha);
        XCPlat(i) = Tpad(end)/settings.tb*(XCPemptyLat - XCPfullLat) + XCPfullLat;
    else
        XCPlat(i) = 5;
    end
end

XCPconstraining = min([XCPlon; XCPlat]); % taking the most constraining one

ceq = [];

c = settings.minStabilityMargin - XCPconstraining;