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
run Config.m

%% RETRIVING DATCOM CONFIG DATA
run ConfigDatcom.m

%% COMPUTATIONAL TIME ESTIMATION
N1 = length(datcom.design.Chord1);
N2 = length(datcom.design.Chord2);
N3 = length(datcom.design.Heigth);
Ns = length(datcom.design.Shape);
No = length(datcom.design.Lnose);
Nt = length(datcom.design.OgType);

DatcomDummy = datcom;
DatcomDummy.design.Chord1 = datcom.design.Chord1(end);
DatcomDummy.design.Chord2 = datcom.design.Chord2(ceil(N2/2));
DatcomDummy.design.Heigth = datcom.design.Heigth(ceil(N3/2));
DatcomDummy.design.Shape = datcom.design.Shape(1);

[~, DummyTime] = AutoMatricesFins(DatcomDummy, false);

seq1 = repelem(1:N1, N2);
seq2 = repmat(1:N2, 1, N1);
N1N2 = length(nonzeros(datcom.design.Chord1(seq1) > datcom.design.Chord2(seq2)));

EstimTime = (N1N2*N3*Ns + No*Nt)*1.1*DummyTime; 

h = floor(EstimTime/3600);
m = floor((EstimTime - h*3600)/60);
Message = strcat('The estimated time to complete the process is', " ",...
     num2str(h), " ", 'hours and', " ", num2str(m), " ",...
     'minutes would you like to continue?');
answer = questdlg(Message, 'Important', 'Yes', 'No', 'Yes');

if isempty(answer) || strcmp(answer, 'No')
    return
end

%% COMPUTING AERODYNAMIC DATA
[dataFins, AMFinstime] = AutoMatricesFins(datcom, true);

%% LAUNCHPAD DYNAMICS
% State
X0pad = [0; 0; 0; 0; settings.m0];
% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180,'ZYX')';

data_ith = dataFins{1};    [settings] = ManageData(data_ith, settings);

[Tpad, Ypad] = ode113(@LaunchPadFreeDyn, [0, 10], X0pad, settings.ode.optionspad, settings, Q0);
AlphaStab = ceil(atan(settings.wind.Mag/Ypad(end, 4))*180/pi);

%% COMPUTING THE LAUNCHPAD STABILITY DATA
datcom.s.Mach = Ypad(end, 4)/340;
datcom.s.Alpha = [(AlphaStab-1), AlphaStab, (AlphaStab+1)];
datcom.s.Beta = 0;
datcom.s.Alt = 0;

[DataPad, ~] = AutoMatricesFins(datcom, false);

%% FINSETs SIMULATION 
tic

% preallocation 
n = length(dataFins);
ApogeeFins = zeros(n, 1);
XCP_pad = zeros(n, 1);

for i = 1:n
    data_ith = dataFins{i};
    xcpf = -DataPad{i}.full.Coeffs.X_C_P(2);
    xcpe = -DataPad{i}.empty.Coeffs.X_C_P(2);
    XCP_pad(i) = Tpad(end)/settings.tb*(xcpe - xcpf) + xcpf;
    if XCP_pad(i) > settings.cal_min
        clc
        fprintf('----------------- Fins Aerodynamics Prediction ----------------- \n')
        fprintf(' Progress %d %% \n\n', 100);
        fprintf('-----------------       Fins Simulation         ----------------- \n')
        fprintf(' Progress %d %% \n\n', floor(i*100/n));
        [settings] = ManageData(data_ith, settings);
        ApogeeFins(i) = RunSim(settings);
    end
end

clc
fprintf('----------------- Fins Aerodynamics Prediction ----------------- \n')
fprintf(' Progress %d %% \n\n', 100);
fprintf('-----------------       Fins Simulation         ----------------- \n')
fprintf(' Progress %d %% \n\n', 100);


%% FINs OPTIMIZATION
% chosing the best apogee
[ApoFinMax, BestFin] = max(ApogeeFins);
FOtime = toc;

%% OGIVE OPTIMIZATION
run ConfigDatcom.m
datcom.design.Chord1 = dataFins{BestFin}.c_max;              
datcom.design.Chord2 = dataFins{BestFin}.c_min;
datcom.design.Heigth = dataFins{BestFin}.h;
datcom.design.shape = dataFins{BestFin}.shape;
[dataOgive, AMOgivetime] = AutoMatricesOgive(datcom);

clc
fprintf('----------------- Fins Aerodynamics Prediction ----------------- \n')
fprintf(' Progress %d %% \n\n', 100);
fprintf('-----------------       Fins Simulation         ----------------- \n')
fprintf(' Progress %d %% \n\n', 100);
fprintf('----------------- Ogive Aerodynamics Prediction ----------------- \n')
fprintf(' Progress %d %% \n\n', 100);

%% OGIVEs SIMULATION 
tic

% preallocation 
n = length(dataOgive);
ApogeeOgive = zeros(n, 1);

for i = 1:n
    clc
    fprintf('----------------- Fins Aerodynamics Prediction ----------------- \n')
    fprintf(' Progress %d %% \n\n', 100);
    fprintf('-----------------       Fins Simulation         ----------------- \n')
    fprintf(' Progress %d %% \n\n', 100);
    fprintf('----------------- Ogive Aerodynamics Prediction ----------------- \n')
    fprintf(' Progress %d %% \n\n', 100);
    fprintf('-----------------       Ogive Simulation        ----------------- \n')
    fprintf(' Progress %d %% \n\n\n\n', floor(i*100/n));
    data_ith = dataOgive{i};
    [settings] = ManageData(data_ith, settings);
    ApogeeOgive(i) = RunSim(settings);
end

%% OGIVEs OPTIMIZATION
% chosing the best apogee
[ApoOgMax, BestOg] = max(ApogeeOgive);
OOtime = toc;

%% print results 
fprintf('COMPUTATIONAL EFFORT: \n\n')
fprintf('- AutoMatrices time, %g [s]\n', AMFinstime + AMOgivetime)
fprintf('- Simulations time, %g [s]\n', FOtime + OOtime)
fprintf('- Total time, %g [s]\n\n\n', FOtime + OOtime + AMFinstime + AMOgivetime)
fprintf('BEST FINSET: \n\n')
fprintf('- shape, %s \n', dataFins{BestFin}.shape)
fprintf('- attached chord, %g [m] \n', dataFins{BestFin}.c_max)
fprintf('- free chord, %g [m] \n', dataFins{BestFin}.c_min)
fprintf('- height, %g [m] \n\n\n', dataFins{BestFin}.h)
fprintf('BEST OGIVE: \n\n')
fprintf('- Type, %s \n', dataOgive{BestOg}.Type)
fprintf('- NoseCone Length, %g [m] \n', dataOgive{BestOg}.Lnose)
if isfield(dataOgive{BestOg}, 'Power')
    fprintf('- Power, %g [m] \n', dataOgive{BestOg}.Power)
end
fprintf('OPTIM ROCKET RESULTS: \n\n')
fprintf('- apogee, %g [m]: \n', ApoOgMax)
fprintf('- stability margin @launchpad exit, %g \n', XCP_pad(BestFin, 1))
