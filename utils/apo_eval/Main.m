%%% Main %%%

clear 
close all
clc 

path = genpath(pwd);
addpath(path);

%% DATA
% principal data 
run config.m

% data of the analyis 
ms = (20:1:21); % structural mass without case+propellant
n_mass = length(ms);
n_motors = size(settings.motors,2);

%% RUN 
tic
% preallocation 
apogee = zeros(n_mass,n_motors);
max_a = zeros(n_mass,n_motors);
Vexit = zeros(n_mass,n_motors);

% simulation runs
for j = 1:n_motors
    
    settings.motor.exp_time = settings.motors(j).t;
    settings.motor.exp_thrust = settings.motors(j).T;
    settings.motor.exp_m = settings.motors(j).m;
    settings.mp = settings.motors(j).mp;                                % [kg]   Propellant Mass
    settings.mm = settings.motors(j).mm;
    settings.tb = settings.motor.exp_time(end);                         % [s]    Burning time
    settings.mfr = settings.mp/settings.tb;                             % [kg/s] Mass Flow Rate
    
    for i = 1:n_mass
        
        settings.ms=ms(i);
        settings.m0=ms(i)+settings.mm;
        
        [apogee(i,j), max_a(i,j)]=start_simulation(settings);
         % mettere eventuale controllo sul vect_XCP
    end
    
end

toc
%% PLOT 

% plot apogee
labels = cell(1,n_motors);
figure()
hold on
grid on
for j = 1:n_motors
    
    plot(ms,apogee(:,j),'o-')
    labels{1,j} = settings.motors(j).MotorName;
    
end
plot(ms,3000.*ones(1,n_mass),'--r','Linewidth',2)
legend(labels)
xlabel('structural mass [kg]')
ylabel('apogee [m]')

% plot max acceleration 
figure()
hold on
for j = 1:n_motors
    
    plot(ms,max_a(:,j),'o-')
    
end
legend(labels)
xlabel('structural mass [kg]')
ylabel('max |a| [g]')







