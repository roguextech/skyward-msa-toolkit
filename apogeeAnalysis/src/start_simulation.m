function [apogee, max_a]=start_simulation(settings)
%{

START_SIMULATION - This function runs the ascent phase simulation using the
ode function "ascent".

INPUTS:
            - settings, settings struct built in config.m

OUTPUTS:
            - apogee, apogee reached by the simulation;
            - max_a, max acceleration reached by the simulation.

Author: Matteo Pozzoli
Skyward Experimental Rocketry | AFD Dept
email: matteo.pozzoli@skywarder.eu
Release date: 23/11/2020

%}

%% STARTING CONDITIONS

% Attitude
Q0 = angle2quat(settings.PHI,settings.OMEGA,0*pi/180,'ZYX')';

% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';
X0a = [X0;V0;W0;Q0;settings.Ixxf;settings.Iyyf;settings.Izzf];

%% WIND GENERATION

[uw,vw,ww,~] = wind_const_generator(settings.wind.Az,settings.wind.Az,...
    settings.wind.El,settings.wind.El,settings.wind.Mag,...
    settings.wind.Mag);

tf = settings.ode.final_time;

%% ASCENT

[Ta,Ya] = ode113(@ascent,[0,tf],X0a,settings.ode.optionsasc1,...
    settings,uw,vw,ww);

%% FINAL STATE ASSEMBLING

% Total State
Y = [Ya(:,1:3) quatrotate(quatconj(Ya(:,10:13)),Ya(:,4:6)) Ya(:,7:13)];
% Total Time
T = Ta;


%% EVALUATE OUTPUT QUANTITIES  

N = length(Y(:,1));

% APOGEE 

apogee = max(-Y(:,3)); % position, index of position at apogee

% VELOCITIES

u = Y(:,4);
v = Y(:,5);
w = -Y(:,6);

% ACCELERATIONS

% main derivatives
ax = (u(3:N)-u(1:N-2))./(T(3:N)-T(1:N-2));
ay = (v(3:N)-v(1:N-2))./(T(3:N)-T(1:N-2));
az = (w(3:N)-w(1:N-2))./(T(3:N)-T(1:N-2));

% add derivative at the boundaries
ax = [u(2)/T(2); ax; (u(end)-u(end-1))/(T(end)-T(end-1))];
ay = [v(2)/T(2); ay; (v(end)-v(end-1))/(T(end)-T(end-1))];
az = [w(2)/T(2); az; (w(end)-w(end-1))/(T(end)-T(end-1))];

A = [ax, ay, az];

% MAXIMUM  ACCELERATIONS
abs_A = vecnorm(A');
max_a = max(abs_A);
max_a=max_a/9.80665;


