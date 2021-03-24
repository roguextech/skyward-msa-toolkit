function dY = LaunchPadFreeDyn(t, Y, settings, Q0, CA)
%{
ODE-Function to compute the dynamics on the launchpad
State = ( x y z | u | m )

(x y z): NED Earth's Surface Centered Frame ("Inertial") coordinates
u: body frame velocity
m: mass
    
Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 18/10/2019
%}

% x = Y(1);
% y = Y(2);
% z = Y(3);
  u = Y(4);

%% QUATERION ATTITUDE
Q_conj = [Q0(1) -Q0(2:4)'];

% Body to Inertial velocities
Vels = quatrotate(Q_conj, [u 0 0]);

%% CONSTANTS
[~, ~, ~, rho] = atmosisa(settings.z0);
S = settings.S;              % [m^2] cross surface
g = settings.g0;                 % [N/kg] module of gravitational field at zero

OMEGA = settings.OMEGA;   
T = interp1(settings.motor.exp_time, settings.motor.exp_thrust, t);
m = settings.ms + interp1(settings.motor.exp_time, settings.motor.exp_m, t);
%% Dynamics
Fg = m*g*sin(OMEGA);                % [N] force due to the gravity
X = 0.5*rho*u^2*S*CA;
du = (-Fg +T -X)/m;

if T < Fg                           % No velocity untill T = Fg
    du = 0;
end

%% FINAL DERIVATIVE STATE ASSEMBLING
dY(1:3) = Vels;
dY(4) = du;
dY = dY';


