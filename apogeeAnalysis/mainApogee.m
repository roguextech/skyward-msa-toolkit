%{

mainApogee - this is the main script; it runs the simulation that has been 
             chosen in configApogee.m. It computes the apogee, the maximum 
             acceleration and the launchpad exit velocity reached by the 
             rocket changing its structural mass and looping on different 
             motors.

CALLED SCRIPT: simulationsData, configApogee

CALLED FUNCTIONS: quickApogeeOnly

CALLED DATA FILES: -

REVISIONS:
- #0 23/11/2000, Release, Matteo Pozzoli

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
commonFunctionsPath = '../commonFunctions/';
addpath(genpath(commonFunctionsPath));
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
vExit = zeros(nMass,nMotors);

% number of iterations
iTOT = (length(vars.control{1}) + length(vars.control{2}))*nMotors;

% SIMULATION RUNS
% worst/best cases loop
nIT = 0;
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
            settings.motor.expTime = settings.motors(k).t;
            settings.motor.expThrust = settings.motors(k).T;
            settings.motor.expM = settings.motors(k).m;
            settings.mp = settings.motors(k).mp;                                % [kg]   Propellant Mass
            settings.mm = settings.motors(k).mm;
            settings.tb = settings.motor.expTime(end);                         % [s]    Burning time
            settings.mfr = settings.mp/settings.tb;                             % [kg/s] Mass Flow Rate

            for l = 1:nMass
                settings.ms=ms(l) + settings.mm - settings.mp;
                settings.m0=ms(l) + settings.mm;

                [apogee(l,k,j,i), max_a(l,k,j,i), vExit(l,k,j,i)] = quickApogeeOnly(settings);
            end
            clc
            nIT = nIT + 1;
            fprintf('Simulation progress:  %d %%\n',floor(nIT/iTOT*100));
        end
    end
end
toc

%% PLOT

figTitles = ["UPWIND", "DOWNWIND"];
labels = cell(1,nMotors);
iCol = zeros(nMotors);

% color line
colorMap = turbo(nMotors+1);

% sorting for a better plot visualization
[~, indexesApo] = sort(apogee(1,:,1,1),'descend');


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
        iCol(indexesApo) = 1:1:nMotors;
        
        iLab = 1;
        for k = indexesApo
            plot(ms,apogee(:,k,j,i),'o-','color',colorMap(iCol(k),:),'LineWidth',1)
            labels{1,iLab} = settings.motors(k).MotorName;
            iLab = iLab + 1;
        end
        
        plot(ms,3000.*ones(1,nMass),'--r','Linewidth',2.5)
        legend(labels)
        xlabel('structural mass [kg]')
        ylabel('apogee [m]')
        
        % Max acceleration plot
        if settings.accelerationPlot
            subplot(1,4,3)
            hold on, grid on;
            
            % sorting for a better plot visualization
            [~, indexesAcc] = sort(max_a(1,:,j,i),'descend');

            % Motors loop
            for k = indexesAcc
                plot(ms,max_a(:,k,j,i),'o-','color',colorMap(iCol(k),:),'LineWidth',1)
            end
            
            xlabel('structural mass [kg]')
            ylabel('max |a| [g]')
        end
        
        % Launchpad exit velocity plot
        if settings.launchpadVelPlot
            subplot(1,4,4)
            hold on, grid on;
            
            % sorting for a better plot visualization
            [~, indexesVexit] = sort(vExit(1,:,j,i),'descend');

            % Motors loop
            for k = indexesVexit
                plot(ms,vExit(:,k,j,i),'o-','color',colorMap(iCol(k),:),'LineWidth',1)
            end
            
            plot(ms,20.*ones(1,nMass),'--r','Linewidth',2.5)
            xlabel('structural mass [kg]')
            ylabel('launchpad exit velocity [m/s]')
        end
    end
end

clearvars -except settings vars Motors DATA_PATH