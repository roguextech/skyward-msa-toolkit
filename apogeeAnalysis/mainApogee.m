%{

MAIN - this is the main script; it runs the simulation that has been chosen
       in config.m. It computes the apogee and the maximum acceleration
       reached by the rocket changing its structural mass and looping on
       different motors.

Author: Matteo Pozzoli
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: matteo.pozzoli@skywarder.eu
Release date: 23/11/2020

%}

close all
clear 
clc

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));

%% LOAD DATA
dataPath = '../data/';
addpath(dataPath);
simulationsData;
configApogee;

ms = linspace( (settings.mNoMot - vars.msDeviation), (settings.mNoMot + vars.msDeviation), vars.nMass );
nMass = vars.nMass;

clear settings.motor

%% MOTOR DATA
% save in settings the acceptable motors 
j = 1;
for i=1:size(Motors,2)
    
    if Motors(i).Itot > vars.Itot_range(1) && Motors(i).Itot < vars.Itot_range(2)
        settings.motors(j) = Motors(i);
        j = j + 1;
    end
  
end
nMotors = size(settings.motors,2);

%% RUN 
tic

% preallocation 
apogee = zeros(nMass,nMotors);
max_a = zeros(nMass,nMotors);
Vexit = zeros(nMass,nMotors);

% SIMULATION RUNS
% worst/best cases loop
for i = 1:2
    % setting wind data
    settings.wind.Mag = vars.wind.Mag(i);
    settings.wind.Az = vars.wind.Az(i);
    settings.wind.El = vars.wind.El(i);
    
    % aerobrakes
    nAerBrake = length(vars.control{i});
    
    % Aerobrakes loop
    for j = 1:nAerBrake
        % setting aerobrakes height
        settings.control = vars.control{i}(j);
        
        % motors loop
        for k = 1:nMotors

            settings.motor.exp_time = settings.motors(k).t;
            settings.motor.exp_thrust = settings.motors(k).T;
            settings.motor.exp_m = settings.motors(k).m;
            settings.mp = settings.motors(k).mp;                                % [kg]   Propellant Mass
            settings.mm = settings.motors(k).mm;
            settings.tb = settings.motor.exp_time(end);                         % [s]    Burning time
            settings.mfr = settings.mp/settings.tb;                             % [kg/s] Mass Flow Rate

            for l = 1:nMass

                settings.ms=ms(l) + settings.mm - settings.mp;
                settings.m0=ms(l) + settings.mm;

                [apogee(l,k,j,i), max_a(l,k,j,i), vExit(l,k,j,i)] = start_simulation(settings);
            end
        end
    end
end

toc

%% PLOT

figTitles = ["UPWIND", "DOWNWIND"];
labels = cell(1,nMotors);

% plot worst/best case
for i = 1:2
    nAerBrake = length(vars.control{i});
    
    % Aerobrakes loop
    for j = 1:nAerBrake
        tit = strcat(figTitles{i},"  ||  ",num2str((vars.control{i}(j)-1)/2*100),"% BRAKES");
        figure('Name',tit,'NumberTitle','off');
        title(tit)
        
        % Apogee plot
        if settings.accelerationPlot
            subplot(1,4,1:2)
        end
        hold on, grid on;
       
        % Motors loop
        for k = 1:nMotors
            plot(ms,apogee(:,k,j,i),'o-')
            labels{1,k} = settings.motors(k).MotorName;
        end
        
        plot(ms,3000.*ones(1,nMass),'--r','Linewidth',2)
        legend(labels)
        xlabel('structural mass [kg]')
        ylabel('apogee [m]')
        
        % Max acceleration plot
        if settings.accelerationPlot
            subplot(1,4,3)
            hold on, grid on;

            % Motors loop
            for k = 1:nMotors
                plot(ms,max_a(:,k,j,i),'o-')
                labels{1,k} = settings.motors(k).MotorName;
            end

            legend(labels)
            xlabel('structural mass [kg]')
            ylabel('max |a| [g]')
        end
        if settings.launchpadVelPlot
            subplot(1,4,4)
            hold on, grid on;

            % Motors loop
            for k = 1:nMotors
                plot(ms,vExit(:,k,j,i),'o-')
                labels{1,k} = settings.motors(k).MotorName;
            end
            
            plot(ms,20.*ones(1,nMass),'--r','Linewidth',2)
            legend(labels)
            xlabel('structural mass [kg]')
            ylabel('launchpad exit velocity [m/s]')
        end
    end
end

clearvars -except settings vars Motors DATA_PATH







