function [dY, parout] = descentDrogue(t, Y, settings)
%{
    descentDrogue - ode function of the drogue's opening phase (first stage of descent)
   
    INPUTS:
        -  t            [nx1]          integration time                [s]
        -  Y            [nx22]         state vector                    [-]

           State vector: [ x y z | u v w | p q r | q0 q1 q2 q3 | Ixx Iyy Izz | x_para y_para z_para | u_para v_para w_para]:

                            * (x y z), NED{north, east, down} horizontal frame;
                            * (u v w), body frame velocities;
                            * (p q r), body frame angular rates;
                            * (q0 q1 q2 q3), attitude unit quaternion;
                            * (Ixx Iyy Izz), Inertias;
                            * (x_para y_para z_para), NED{north, east, down} horizontal frame of the drogue;
                            * (u_para v_para w_para), body frame velocities of the drogue;

        -  settings     [1x1 struct]   settings struct                 [-]

    OUTPUTS:
        -  dY           [nx22]         state derivatives vector        [-]
        -  parout       [1x1 struct]   quantities saved fot the plots  [-]

    REVISIONS:
        - #0 16/12/2020, Release, Davide Rosato, Fiammetta Artioli
%}

%% RECALLING THE STATE
% Rocket state
x_rocket = Y(1);
y_rocket = Y(2);
z_rocket = Y(3);
u_rocket = Y(4);
v_rocket = Y(5);
w_rocket = Y(6);
p = Y(7);
q = Y(8);
r = Y(9);
q0 = Y(10);
q1 = Y(11);
q2 = Y(12);
q3 = Y(13);

% parachute state
x_para = Y(17);
y_para = Y(18);
z_para = Y(19);
u_para = Y(20);
v_para = Y(21);
w_para = Y(22);

%% CONSTANTS
g = settings.g0/(1 + (-z_rocket*1e-3/6371))^2;
para = settings.paraN;
t0p = settings.t0p;

% Parachute parameters
S_para = settings.para(para).S;                                         
CD_para = settings.para(para).CD;
D0 = sqrt(4*settings.para(para).S/pi);
SCD0 = S_para*CD_para;

% Mass
m_rocket = settings.ms - settings.para(para(1)).mass;
m_para = settings.mnc + settings.para(para).mass;

% OMEGA = settings.OMEGA;            
uncert = [0, 0];
if not(settings.wind.input) && not(settings.wind.model)
    uw = settings.constWind(1); vw = settings.constWind(2); ww = settings.constWind(3);
end

% Inertias
Ixx = settings.Ixxe;
Iyy = settings.Iyye;
Izz = settings.Izze;

%% QUATERNION ATTITUDE
Q = [q0 q1 q2 q3];
Q = Q/norm(Q);

%% ADDING WIND (supposed to be added in NED axes);
if settings.wind.model
    
    [uw, vw, ww] = windMatlabGenerator(settings, z_rocket, t);
    
elseif settings.wind.input
    
    [uw, vw, ww] = windInputGenerator(settings, z_rocket, uncert);
    
end

dcm = quatToDcm(Q);
wind = dcm*[uw; vw; ww];

% Rocket (BODY) relative velocities (plus wind);
ur_rocket = u_rocket - wind(1);
vr_rocket = v_rocket - wind(2);
wr_rocket = w_rocket - wind(3);

% Body to Inertial velocities
Vels_rocket = dcm'*[u_rocket; v_rocket; w_rocket];
V_norm_rocket = norm([ur_rocket; vr_rocket; wr_rocket]);

% Parachute (NED) relative velocities (plus wind) 
ur_para = u_para - uw;
vr_para = v_para - vw;
wr_para = w_para - ww;

Vels_para = [u_para; v_para; w_para];
Vrel_para = [ur_para; vr_para; wr_para];
V_norm_para = norm([ur_para; vr_para; wr_para]);

%% PARACHUTE REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity
if V_norm_para < 1e-3
    t_vers = [0; 0; -1];
else
    t_vers = - Vrel_para/V_norm_para;
end

%% ATMOSPHERE DATA
% since z_rocket is similar to z_para, atmospherical data will be computed
% on z_rocket
absoluteAltitude = -z_rocket + settings.z0;
[~, a, P, rho] = atmosisa(absoluteAltitude);
M = V_norm_rocket/a;
M_value = M;

%% RELATIVE POSITION AND VELOCITY VECTORS
% (NED) positions of parachute and rocket
posPara = [x_para; y_para; z_para]; 
posRocket = [x_rocket; y_rocket; z_rocket];
xcg = settings.xcg(2);

% (NED) position of the point from where the parachute is deployed
posDepl = posRocket + dcm'*[(xcg-settings.Lnose); 0; 0];

% (NED) relative position between parachute and rocket
posRel = posPara - posDepl;

if norm(posRel) < 1e-3
    posRel_vers = [0; 0; -1];
else
    posRel_vers = posRel/norm(posRel);
end

% (NED) velocities of parachute and rocket
velPara = [u_para; v_para; w_para]; 
velRocket = dcm'*[u_rocket; v_rocket; w_rocket];

% (NED) velocity of the point from where the parachute is deployed
velDepl = velRocket + dcm'*cross([p; q; r],[(xcg-settings.Lnose); 0; 0]);

% (NED) relative velocity between parachute and rocket
velRel = velPara - velDepl;

% Relative velocity projected along the shock chord
velRel_chord = dot(velRel,posRel_vers);

%% CHORD TENSION (ELASTIC-DAMPING MODEL)
if norm(posRel) > settings.para(para).L  
    T_chord = (norm(posRel) - settings.para(para).L)* settings.para(para).K +...
        velRel_chord * settings.para(para).C;
else
    T_chord = 0;
end

%% PARACHUTE FORCES
% computed in the NED-frame reference system
t0 = settings.para(para).nf * D0/V_norm_para;
tx = t0 * settings.para(para).CX^(1/settings.para(para).m);

dt = t-t0p;

if dt < 0
    SCD_para = 0;
elseif dt < tx
    SCD_para = SCD0 * (dt/t0)^settings.para(para).m;
else
    SCD_para = SCD0 * (1+(settings.para(para).CX-1)*exp(-2*(dt-tx)/t0));
end

D_para = 0.5*rho*V_norm_para^2*SCD_para*t_vers;
Fg_para = [0; 0; m_para*g];                                                 

Ft_chord_para = -T_chord * posRel_vers;                                    
F_para = D_para + Fg_para + Ft_chord_para;                         

%% ROCKET FORCES
% computed in the body-frame reference system
Fg_rocket = dcm*[0; 0; m_rocket*g];                        

Ft_chord_rocket = T_chord * dcm*posRel_vers;              
F_rocket = Fg_rocket + Ft_chord_rocket;

%% ROCKET STATE DERIVATIVES
% velocity (BODY frame)
du_rocket = F_rocket(1)/m_rocket-q*w_rocket+r*v_rocket;
dv_rocket = F_rocket(2)/m_rocket-r*u_rocket+p*w_rocket;
dw_rocket = F_rocket(3)/m_rocket-p*v_rocket+q*u_rocket;

% Rotation
b = [(xcg-settings.Lnose) 0 0];
Momentum = cross(b, Ft_chord_rocket);                                   

dp_rocket = (Iyy-Izz)/Ixx*q*r + Momentum(1)/Ixx;
dq_rocket = (Izz-Ixx)/Iyy*p*r + Momentum(2)/Iyy;
dr_rocket = (Ixx-Iyy)/Izz*p*q + Momentum(3)/Izz;

% Quaternion
OM = [ 0 -p -q -r  ;
       p  0  r -q  ;
       q -r  0  p  ;
       r  q -p  0 ];

dQQ = 1/2*OM*Q';

%% PARACHUTE STATE DERIVATIVES
% velocity (NED frame)
du_para = F_para(1)/m_para;
dv_para = F_para(2)/m_para;
dw_para = F_para(3)/m_para;

%% FINAL DERIVATIVE STATE ASSEMBLING
dY(1:3) = Vels_rocket;
dY(4) = du_rocket;
dY(5) = dv_rocket;
dY(6) = dw_rocket;
dY(7) = dp_rocket;
dY(8) = dq_rocket;
dY(9) = dr_rocket;
dY(10:13) = dQQ;
dY(14:16) = [p q r];
dY(17:19) = Vels_para;
dY(20) = du_para;
dY(21) = dv_para;
dY(22) = dw_para;
dY = dY';

%% SAVING THE QUANTITIES FOR THE PLOTS
parout.integration.t = t;

parout.interp.M = M_value;
parout.interp.alt = -z_rocket;

parout.wind.NED_wind = [uw, vw, ww];
parout.wind.body_wind = wind;

parout.velocities = Vels_rocket;

parout.air.rho = rho;
parout.air.P = P;

parout.accelerations.body_acc = [du_rocket, dv_rocket, dw_rocket];
parout.accelerations.ang_acc = [dp_rocket, dq_rocket, dr_rocket];

parout.forces.T_chord = [T_chord, NaN];
parout.forces.D = [norm(D_para), NaN];

parout.SCD = [SCD_para/SCD0, NaN];