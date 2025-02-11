function [dY, parout] = descentBal(t, Y, settings)

%{
descentBal - ode function of the ballistic descent

INPUTS:
- t,         double [1, 1], integration time [s];
- Y,         double [16, 1], state vector [ x y z | u v w | p q r | q0 q1 q2 q3 | Ixx Iyy Izz ]:

                                * (x y z), NED{north, east, down} horizontal frame;
                                * (u v w), body frame velocities;
                                * (p q r), body frame angular rates;
                                * (q0 q1 q2 q3), attitude unit quaternion;
                                * (Ixx Iyy Izz), Inertias;
- settings, struct(motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind), rocket data structure;

OUTPUTS:
- dY ,       double [13, 1] state derivatives
- parout,    struct, interesting fligth quantities structure (aerodyn coefficients, forces and so on..)


CALLED FUNCTIONS: windMatlabGenerator, windInputGenerator, quatToDcm, interpCoeffs 

NOTE: To get the NED velocities the body-frame must be multiplied for the
conjugated of the current attitude quaternion
E.G.  quatrotate(quatconj(Y(:,10:13)),Y(:,4:6))

REVISIONS:
-#0 31/12/2014, Release, Ruben Di Battista

-#1 16/04/2016, Second version, Francesco Colombi

-#2 13/01/2018, Third version, Adriano Filippo Inno

%}

% recalling the state
% x = Y(1);
% y = Y(2);
z = Y(3);
u = Y(4);
v = Y(5);
w = Y(6);
p = Y(7);
q = Y(8);
r = Y(9);
q0 = Y(10);
q1 = Y(11);
q2 = Y(12);
q3 = Y(13);
m = settings.ms;
Ixx = settings.Ixxe;
Iyy = settings.Iyye;
Izz = settings.Izze;

%% CONSTANTS
% Everything related to empty condition (descent-fase)
S = settings.S;                         % [m^2] cross surface
C = settings.C;                         % [m]   caliber
g = settings.g0/(1 + (-z*1e-3/6371))^2; % [N/kg] module of gravitational field
T = 0;

if settings.stoch.N > 1
    uncert = settings.stoch.uncert;
    Day = settings.stoch.Day;
    Hour = settings.stoch.Hour;
    uw = settings.stoch.uw; vw = settings.stoch.vw; ww = settings.stoch.ww;
else       
    uncert = settings.wind.inputUncertainty;
    uw = settings.constWind(1); vw = settings.constWind(2); ww = settings.constWind(3);
end

%% QUATERION ATTITUDE
Q = [q0 q1 q2 q3];
Q = Q/norm(Q);

%% ADDING WIND (supposed to be added in NED axes);
if settings.wind.model
    
    if settings.stoch.N > 1
        [uw, vw, ww] = windMatlabGenerator(settings, z, t, Hour, Day);
    else
        [uw, vw, ww] = windMatlabGenerator(settings, z, t);
    end
    
elseif settings.wind.input
    [uw, vw, ww] = windInputGenerator(settings, z, uncert);
end

dcm = quatToDcm(Q);
wind = dcm*[uw; vw; ww];

% Relative velocities (plus wind);
ur = u - wind(1);
vr = v - wind(2);
wr = w - wind(3);

% Body to Inertial velocities
Vels = dcm'*[u; v; w];
V_norm = norm([ur vr wr]);
    
%% ATMOSPHERE DATA
if -z < 0     % z is directed as the gravity vector
    z = 0;
end

absoluteAltitude = -z + settings.z0;
[~, a, P, rho] = atmosphereData(absoluteAltitude, g);

M = V_norm/a;
M_value = M;

%% AERODYNAMICS ANGLES
if not(ur < 1e-9 || V_norm < 1e-9)
    alpha = atan(wr/ur);
    beta = atan(vr/ur);             % beta = asin(vr/V_norm); is the classical notation, Datcom uses this one though. 
else
    alpha = 0;
    beta = 0;
end

alpha_value = alpha;
beta_value = beta;

%% CHOSING THE CONDITION VALUE
% interpolation of the coefficients with the value in the nearest condition of the Coeffs matrix

c = 1; % descent with no aerobrakes

%% INTERPOLATE AERODYNAMIC COEFFICIENTS:
[coeffsValues, angle0] = interpCoeffs(t, alpha, M, beta, absoluteAltitude,...
                                        c,settings);

% Retrieve Coefficients 
CA = coeffsValues(1); CYB = coeffsValues(2); CY0 = coeffsValues(3);
CNA = coeffsValues(4); CN0 = coeffsValues(5); Cl = coeffsValues(6);
Clp = coeffsValues(7); Cma = coeffsValues(8); Cm0 = coeffsValues(9);
Cmad = coeffsValues(10); Cmq = coeffsValues(11); Cnb = coeffsValues(12);
Cn0 = coeffsValues(13); Cnr = coeffsValues(14); Cnp = coeffsValues(15);

% compute CN,CY,Cm,Cn (linearized with respect to alpha and beta):
alpha0 = angle0(1); beta0 = angle0(2);

CN = (CN0 + CNA*(alpha - alpha0));
CY = (CY0 + CYB*(beta - beta0));
Cm = (Cm0 + Cma*(alpha - alpha0));
Cn = (Cn0 + Cnb*(beta - beta0));

%% FORCES
% first computed in the body-frame reference system
qdyn = 0.5*rho*V_norm^2;      % [Pa] dynamics pressure
qdynL_V = 0.5*rho*V_norm*S*C; % 

X = qdyn*S*CA;                % [N] x-body component of the aerodynamics force
Y = qdyn*S*CY;                % [N] y-body component of the aerodynamics force
Z = qdyn*S*CN;                % [N] z-body component of the aerodynamics force
Fg = dcm*[0; 0; m*g];         % [N] force due to the gravity

F = Fg +[-X, +Y, -Z]';        % [N] total forces vector

%% STATE DERIVATIVES
% velocity
du = F(1)/m - q*w + r*v;
dv = F(2)/m - r*u + p*w;
dw = F(3)/m - p*v + q*u;

% Rotation
dp = (Iyy - Izz)/Ixx*q*r + qdynL_V/Ixx*(V_norm*Cl + Clp*p*C/2);
dq = (Izz - Ixx)/Iyy*p*r + qdynL_V/Iyy*(V_norm*Cm + (Cmad + Cmq)*q*C/2);
dr = (Ixx - Iyy)/Izz*p*q + qdynL_V/Izz*(V_norm*Cn + (Cnr*r + Cnp*p)*C/2);

% Quaternion
OM = 1/2* [ 0 -p -q -r  ;
            p  0  r -q  ;
            q -r  0  p  ;
            r  q -p  0 ];

dQQ = OM*Q'; 

%% FINAL DERIVATIVE STATE ASSEMBLING
dY(1:3) = Vels;
dY(4) = du;
dY(5) = dv;
dY(6) = dw;
dY(7) = dp;
dY(8) = dq;
dY(9) = dr;
dY(10:13) = dQQ;
dY = dY';

%% SAVING THE QUANTITIES FOR THE PLOTS
parout.integration.t = t;

parout.interp.M = M_value;
parout.interp.alpha = alpha_value;
parout.interp.beta = beta_value;
parout.interp.alt = -z;

parout.wind.NED_wind = [uw, vw, ww];
parout.wind.body_wind = wind;

parout.rotations.dcm = dcm;

parout.velocities = Vels;

parout.forces.AeroDyn_Forces = [X, Y, Z];
parout.forces.T = T;

parout.air.rho = rho;
parout.air.P = P;

parout.accelerations.body_acc = [du, dv, dw];
parout.accelerations.ang_acc = [dp, dq, dr];

parout.forces.AeroDyn_Forces = [X, Y, Z];
parout.forces.T = T;
parout.coeff.CA = CA;
parout.coeff.CYB = CYB;
parout.coeff.CNA = CNA;
parout.coeff.Cl = Cl;
parout.coeff.Clp = Clp;
parout.coeff.Cma = Cma;
parout.coeff.Cmad = Cmad;
parout.coeff.Cmq = Cmq;
parout.coeff.Cnb = Cnb;
parout.coeff.Cnr = Cnr;
parout.coeff.Cnp = Cnp;

end