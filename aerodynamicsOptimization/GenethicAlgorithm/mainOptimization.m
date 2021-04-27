%{
configOptimization - this script runs the aerodynamics surfaces optimization.

CALLED SCRIPTS: simulationData, configOptimization

CALLED FUNCTIONS: optimizationGA, XCPcheck

CALLED DATA FILES: /

REVISIONS:
- 0     21/10/20,   release     Adriano Filippo Inno
%}

clear 
close all
clc 

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));

datcomPath = '../../commonFunctions/Datcom/';
if ismac
    if ~isfile(fullfile(datcomPath, 'datcom'))
        error('datcom is missing')
    end
else
    if ~isfile(fullfile(datcomPath, 'datcom.exe'))
        error('datcom.exe is missing')
    end
end

%% LOAD DATA
dataPath = '../../data/';
addpath(dataPath);
commonFunctionsPath = '../../commonFunctions/';
addpath(genpath(commonFunctionsPath));
simulationsData;
configOptmization;

%% OPTIMIZATION
tic

IntCon = 1:6;
options = optimoptions('ga', 'MaxStallGenerations', 5, 'FunctionTolerance', ...
    1/expectedApogee, 'MaxGenerations', 1000, 'NonlinearConstraintAlgorithm', 'penalty',...
    'PopulationSize', 400, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter');
nonlcon = @(x) XCPcheck(x, datcom, settings);
fitnessfcn = @(x) optimizationGA(x, datcom, settings);
[x, fval, exitflag] = ga(fitnessfcn, 6, A, b, [], [],...
    lb, ub, nonlcon, IntCon, options);

computationalTime = toc;

XCP = -XCPcheck(x, datcom, settings) + settings.minStabilityMargin;

cd(datcomPath)
delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat');


%% print results 
fprintf('COMPUTATIONAL EFFORT: \n\n')
fprintf('- Total time, %g [s]\n\n\n', computationalTime)
fprintf('FINS RESULTS: \n\n')
if x(4) == 1
    fprintf('- shape, %s \n', 'isoscele')
elseif x(4) == 2
    fprintf('- shape, %s \n', 'rectangular')
elseif x(4) == 3
    fprintf('- shape, %s \n', 'paralleloid')
end

fprintf('- attached chord, %d [cm] \n', x(1))
fprintf('- free chord, %d [cm] \n', x(2))
fprintf('- height, %d [cm] \n\n\n', x(3))
fprintf('BEST OGIVE: \n\n')

if x(6) == 1
    fprintf('- Type, %s \n', 'Karman')
elseif x(6) == 2
    fprintf('- Type, %s \n', 'Haack')
elseif x(6) == 3
    fprintf('- Type, %s \n', 'Ogive')
elseif x(6) == 4
    fprintf('- Type, %s \n', 'Power 1/3')
elseif x(6) == 5
    fprintf('- Type, %s \n', 'Power 1/3')
elseif x(6) == 6
    fprintf('- Type, %s \n', 'Power 1/3')
end

fprintf('- NoseCone Length, %d [cm] \n\n\n', x(5))
fprintf('OPTIM ROCKET RESULTS: \n\n')
fprintf('- apogee, %g [m]: \n', -fval)
fprintf('- stability margin @launchpad exit, %g \n', XCP)

