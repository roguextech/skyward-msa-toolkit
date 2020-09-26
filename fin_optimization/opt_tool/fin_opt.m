%{

FIN_OPT - This script chose the best fin set between the chosen.
For every set, missile datcom is used to predict the aerodynamic coefficients.

Author: Matteo Pozzoli
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: matteo.pozzoli@skywarder.eu
Website: http://www.skywarder.eu
Release date: 14/10/2019

%}

clear 
close all
clc 

path = genpath(pwd);
addpath(path);

%% RETRIVING GEOMETRICAL DATA
run config.m

%% RETRIVING DATCOM CONFIG DATA
run config_datcom.m

%% COMPUTING AERODYNAMIC DATA
[data, AMtime] = Auto_Matrices(datcom);

%% LAUNCHPAD DYNAMICS
% State
X0pad = [0; 0; 0; 0; settings.m0];
% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180,'ZYX')';

data_ith = data{1};    [settings] = ManageData(data_ith, settings);

[Tpad, Ypad] = ode113(@LaunchPadFreeDyn, [0, 10], X0pad, settings.ode.optionspad, settings, Q0);
AlphaStab = ceil(atan(settings.wind.Mag/Ypad(end, 4))*180/pi);

%% COMPUTING THE LAUNCHPAD STABILITY DATA
datcom.s.Mach = Ypad(end, 4)/340;
datcom.s.Alpha = [(AlphaStab-1), AlphaStab, (AlphaStab+1)];
datcom.s.Beta = 0;
datcom.s.Alt = 0;

[DataPad, ~] = Auto_Matrices(datcom);

%% FINSETs SIMULATION 
tic

% preallocation 
n = length(data);
apogee = zeros(n, 1);
XCP_pad = zeros(n, 1);

for i = 1:n
    data_ith = data{i};
    xcpf = -DataPad{i}.full.Coeffs.X_C_P(2);
    xcpe = -DataPad{i}.empty.Coeffs.X_C_P(2);
    XCP_pad(i) = Tpad(end)/settings.tb*(xcpe - xcpf) + xcpf;
    if XCP_pad(i) > settings.cal_min
        [settings] = ManageData(data_ith, settings);
        apogee(i) = run_sim(settings);
    end
end

%% OPTIMIZATION
% chosing the best apogee
[apo_max, best] = max(apogee);
FOtime = toc;

%% print results 
fprintf('COMPUTATIONAL EFFORT: \n\n')
fprintf('- AutoMatrices time, %g [s]\n', AMtime)
fprintf('- Simulations time, %g [s]\n', FOtime)
fprintf('- Total time, %g [s]\n\n\n', FOtime + AMtime)
fprintf('BEST FINSET: \n\n')
fprintf('- shape, %s \n', data{best}.shape)
fprintf('- attached chord, %g [m] \n', data{best}.c_max)
fprintf('- free chord, %g [m] \n', data{best}.c_min)
fprintf('- height, %g [m] \n\n\n', data{best}.h)
fprintf('BEST FINSET SIMULATION RESULTS: \n\n')
fprintf('- apogee, %g [m]: \n', apo_max)
fprintf('- stability margin @launchpad exit, %g \n', XCP_pad(best, 1))
