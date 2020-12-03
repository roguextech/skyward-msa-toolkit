%{
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Update date: 21/10/20

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

tic

%% VARIABLES BOUNDARIES
%%% Lower boundary
lb(1) = 25;     % 1 --> chord1         [cm]
lb(2) = 5;     % 2 --> chord2         [cm]
lb(3) = 5;     % 3 --> heigth         [cm]
lb(4) = 3;      % 4 --> Fin type       [/]
lb(5) = 25;     % 5 --> Ogive Length   [cm]
lb(6) = 1;      % 6 --> Ogive Type     [/]

%%% Upper boundary
ub(1) = 50;     % 1 --> chord1         [cm]
ub(2) = 20;     % 2 --> chord2         [cm]
ub(3) = 20;     % 3 --> heigth         [cm]
ub(4) = 3;      % 4 --> Fin type       [/]
ub(5) = 50;     % 5 --> Ogive Length   [cm]
ub(6) = 1;      % 6 --> Ogive Type     [/]

%%% Inequality constraint (A*x < b)
% imposing the fixed chord, x(1), to be greater than the free chord x(2)
% so -x(1) + x(2) < 0
% imposing the fixed chord, x(1), to be greater than the heigth x(3) to
% reduce the flessibility, so -x(1) + x(3) < 0
% 
A = [-1 1 0 0 0 0
     -1 0 1 0 0 0 ];
b = [0; 0];

IntCon = 1:6;
options = optimoptions('ga', 'MaxStallGenerations', 15, 'FunctionTolerance', ...
    1, 'MaxGenerations', 50, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter');
nonlcon = @(x) XCPcheck(x, datcom, settings);
fitnessfcn = @(x) OptimizationGA(x, datcom, settings);
[x, fval, exitflag] = ga(fitnessfcn, 6, A, b, [], [],...
    lb, ub, nonlcon, IntCon, options);

computationalTime = toc;

XCP = -XCPcheck(x, datcom, settings) + settings.cal_min;

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

fprintf('- NoseCone Length, %d [m] \n\n\n', x(5))
fprintf('OPTIM ROCKET RESULTS: \n\n')
fprintf('- apogee, %g [m]: \n', -fval)
fprintf('- stability margin @launchpad exit, %g \n', XCP)

