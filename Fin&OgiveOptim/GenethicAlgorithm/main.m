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
lb(1) = 30;     % 1 --> chord1         [cm]
lb(2) = 10;     % 2 --> chord2         [cm]
lb(3) = 10;     % 3 --> heigth         [cm]
lb(4) = 1;      % 4 --> Fin type       [/]
lb(5) = 30;     % 5 --> Ogive Length   [cm]
lb(6) = 1;      % 6 --> Ogive Type     [/]

%%% Upper boundary
ub(1) = 50;     % 1 --> chord1         [cm]
ub(2) = 20;     % 2 --> chord2         [cm]
ub(3) = 20;     % 3 --> heigth         [cm]
ub(4) = 3;      % 4 --> Fin type       [/]
ub(5) = 50;     % 5 --> Ogive Length   [cm]
ub(6) = 6;      % 6 --> Ogive Type     [/]

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
    1, 'MaxGenerations', 200, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter');
nonlcon = @(x) XCPcheck(x, datcom, settings);
fitnessfcn = @(x) OptimizationGA(x, datcom, settings);
[x, fval, exitflag] = ga(fitnessfcn, 6, A, b, [], [],...
    lb, ub, nonlcon, IntCon, options);

toc

delete('for003.dat', 'for004.dat', 'for005.dat', 'for006.dat', 'for009.dat',...
    'for010.dat', 'for011.dat', 'for012.dat');


