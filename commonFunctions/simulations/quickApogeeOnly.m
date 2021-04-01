function [apogee, maxAccel, vExit] = quickApogeeOnly(settings)

%{

RunSim - This function tests the fins simulating the ascent

Author: Matteo Pozzoli
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: matteo.pozzoli@skywarder.eu
Website: http://www.skywarder.eu
Release date: 14/10/2019

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Update date: 21/10/20

%}

%% STARTING CONDITIONS

% Attitude
Q0 = angleToQuat(settings.PHI, settings.OMEGA, 0*pi/180)';

%% WIND GENERATION

[uw, vw, ww, ~] = windConstGenerator(settings.wind);
settings.constWind = [uw, vw, ww];
tf = settings.ode.final_time;

%% ASCENT
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';
X0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];
[Ta, Ya] = ode113(@ascent, [0, tf], X0a, settings.ode.optionsasc1, settings);

%% CALCULATE OUTPUT QUANTITIES
apogee = -Ya(end, 3);

if nargout == 3
    
    Y = [Ya(:, 1:3) quatrotate(quatconj(Ya(:, 10:13)), Ya(:, 4:6)) Ya(:, 7:13)];
    N = length(Ta);
    
    % VELOCITIES
    u = Y(:,4);
    v = Y(:,5);
    w = -Y(:,6);
    V = [u, v, w];
    
    % ACCELERATIONS
    ax = (u(3:N)-u(1:N-2))./(Ta(3:N)-Ta(1:N-2));
    ay = (v(3:N)-v(1:N-2))./(Ta(3:N)-Ta(1:N-2));
    az = (w(3:N)-w(1:N-2))./(Ta(3:N)-Ta(1:N-2));
    A = [ax, ay, az];
    
    % MAXIMUM  ACCELERATIONS
    abs_A = vecnorm(A');
    maxAccel = max(abs_A)/9.80665;
    
    % LAUNCHPAD EXIT VELOCITY
    abs_V = vecnorm(V');
    X = Y(:,1:3);
    abs_X = vecnorm(X');
    iexit = find(abs_X <= settings.lrampa);  % checking where the missile is undocked from the hook of the launch pad
    iexit = iexit(end);
    vExit = abs_V(iexit);

end