%%% Main %%%

clear 
close all
clc 

%% DATA
% principal data 
run config.m

% data of the analyis 
m0 = 24:1:30;
n_mass = length(m0);
n_motors = size(settings.motors,2);

%% RUN 

% preallocation 
apogee = zeros(n_mass,n_motors);
max_a = zeros(n_mass,n_motors);
Vexit = zeros(n_mass,n_motors);

% simulation runs
for j = 1:n_motors
    
    settings.motor.exp_time = settings.motors(j).t;
    settings.motor.exp_thrust = settings.motors(j).T;
    settings.mp = settings.motors(j).mp;                                                % [kg]   Propellant Mass
    settings.mnc = 0.500;                                               % [kg]   Nosecone Mass
    settings.tb = settings.motor.exp_time(end);                         % [s]    Burning time
    settings.mfr = settings.mp/settings.tb;                             % [kg/s] Mass Flow Rate
    
    for i = 1:n_mass
        
        settings.m0=m0(i);
        setting.ms=m0(i)-settings.mp;
        
        [apogee(i,j), max_a(i,j),Vexit(i,j),t,~]=start_simulation(settings);
         % mettere eventuale controllo sul vect_XCP
    end
    
end


%% PLOT 

% plot apogee
figure()
hold on
for j = 1:n_motors
    
    plot(m0,apogee(:,j),'o-')
    labels{1,j} = settings.motors(j).MotorName;
    
end
plot(m0,3000.*ones(1,n_mass),'--r','Linewidth',2)
legend(labels)
xlabel('structural mass [kg]')
ylabel('apogee [m]')

% plot max acceleration 
figure()
hold on
for j = 1:n_motors
    
    plot(m0,max_a(:,j),'o-')
    
end
legend(labels)
xlabel('structural mass [kg]')
ylabel('max |a| [g]')







