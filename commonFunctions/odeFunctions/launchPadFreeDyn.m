function dY = launchPadFreeDyn(t, Y, settings, Q0, CA)
%{
launchPadFreeDyn - ODE-Function to compute the dynamics on the launchpad

INPUTS:
- t,        double [1, 1], integration time  [s];
- Y,        double [4, 1], integration state, check launchPadFreeDyn for explanation; 
                           Y = ( x y z | u )
                           (x y z): NED Earth's Surface Centered Frame ("Inertial") coordinates
                            u: body frame velocity
- settings, struct (motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind), 
                   simulation data;
- Q0,       double [4, 1], quaternion attitude [/];
- CA,       double [1, 1], axial coefficient, supposed costant [/]

OUTPUTS:
- dY,       double [4, 1], state derivatives;

CALLED FUNCTIONS: /

REVISIONS:
- 0     21/10/20,   release     Adriano Filippo Inno
%}

%% STATES RECALL
% x = Y(1);
% y = Y(2);
% z = Y(3);
  u = Y(4);

%% QUATERION ATTITUDE
conjQ = [Q0(1) -Q0(2:4)'];

% Body to Inertial velocities
Vels = quatrotate(conjQ, [u 0 0]);

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


