function [c, ceq] = XCPcheck(x, datcom, settings)

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

%%
datcom.Mach = 0.05;
datcom.Alpha = [-0.1, 0, 0.1];
datcom.Beta = 0;
datcom.Alt = 0;

%%
%%%
xcg = datcom.xcg;
datcom.xcg = xcg(1) + datcom.Lnose;
createFor006(datcom);
[Coeffs0, ~] = datcomParser5();

%% LAUNCHPAD DYNAMICS
% State
X0pad = [0; 0; 0; 0; settings.m0];
% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';

[Tpad, Ypad] = ode113(@LaunchPadFreeDyn, [0, 10], X0pad, settings.ode.optionspad,...
    settings, Q0, Coeffs0.CA(2));
AlphaStab = ceil(atan(settings.wind.Mag/Ypad(end, 4))*180/pi);

%% COMPUTING THE LAUNCHPAD STABILITY DATA
datcom.Mach = Ypad(end, 4)/340;
datcom.Alpha = [(AlphaStab-1), AlphaStab, (AlphaStab+1)];
datcom.Beta = 0;
datcom.Alt = 0;
%%%
createFor006(datcom);
[CoeffsF, ~] = datcomParser5();
%%%
datcom.xcg = xcg(2) + datcom.Lnose;
createFor006(datcom);
[CoeffsE, ~] = datcomParser5();

xcpf = -CoeffsF.X_C_P(2);
xcpe = -CoeffsE.X_C_P(2);
XCP_pad = Tpad(end)/settings.tb*(xcpe - xcpf) + xcpf;


ceq = [];

c = settings.cal_min - XCP_pad; 