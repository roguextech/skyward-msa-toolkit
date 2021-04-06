function [dY, parout] = descentMain(t, Y, settings)
%{
    descentMain - ode function of the main's descent phase (third stage of descent)
   
    INPUTS:
        -  t, array, [n° variations, 1], integration time              
        -  Y, array, [n° variations, 28], state vector              

           State vector: [ x y z | u v w | p q r | q0 q1 q2 q3 | Ixx Iyy Izz | x_para1 y_para1 z_para1 | u_para1 v_para1 w_para1 | x_para2 y_para2 z_para2 | u_para2 v_para2 w_para2]:

                            * (x y z), NED{north, east, down} horizontal frame;
                            * (u v w), body frame velocities;
                            * (p q r), body frame angular rates;
                            * (q0 q1 q2 q3), attitude unit quaternion;
                            * (Ixx Iyy Izz), Inertias;
                            * (x_para1 y_para1 z_para1), NED{north, east, down} horizontal frame of the drogue;
                            * (u_para1 v_para1 w_para1), body frame velocities of the drogue;
                            * (x_para2 y_para2 z_para2), NED{north, east, down} horizontal frame of the main;
                            * (u_para2 v_para2 w_para2), body frame velocities of the main;

        -  settings, struct, stores data of the rocket and of the simulation                

    OUTPUTS:
        -  dY, array, [n° variations, 28], state derivatives vector        
        -  parout, struct, quantities saved fot the plots  

    CALLED FUNCTIONS:
        - windMatlabGenerator, windInputGenerator, quatToDcm; 

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

% first parachute state
x_para1 = Y(17);
y_para1 = Y(18);
z_para1 = Y(19);
u_para1 = Y(20);
v_para1 = Y(21);
w_para1 = Y(22);

% second parachute state
x_para2 = Y(23);
y_para2 = Y(24);
z_para2 = Y(25);
u_para2 = Y(26);
v_para2 = Y(27);
w_para2 = Y(28);

%% CONSTANTS
g = settings.g0/(1 + (-z_rocket*1e-3/6371))^2;
para = settings.paraN;
t0p = settings.t0p;

% Parachutes parameters
% drogue
S_para1 = settings.para(para(1)).S;                     
CD_para1 = settings.para(para(1)).CD;
D0_1 = sqrt(4*settings.para(para(1)).S/pi);
SCD0_1 = S_para1*CD_para1;
% main
S_para2 = settings.para(para(2)).S;                      
CD_para2 = settings.para(para(2)).CD;
D0_2 = sqrt(4*settings.para(para(2)).S/pi);
SCD0_2 = S_para2*CD_para2;

% Mass
m_rocket = settings.ms - settings.para(para(1)).mass - settings.para(para(2)).mass;
m_para1 = settings.mnc + settings.para(para(1)).mass;
m_para2 = settings.para(para(2)).mass;

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

% Parachute 1 (NED) relative velocities (plus wind) 
ur_para1 = u_para1 - uw;
vr_para1 = v_para1 - vw;
wr_para1 = w_para1 - ww;

Vels_para1 = [u_para1; v_para1; w_para1];
Vrel_para1 = [ur_para1; vr_para1; wr_para1];
V_norm_para1 = norm([ur_para1; vr_para1; wr_para1]);

% Parachute 2 (NED) relative velocities (plus wind) 
ur_para2 = u_para2 - uw;
vr_para2 = v_para2 - vw;
wr_para2 = w_para2 - ww;

Vels_para2 = [u_para2; v_para2; w_para2];
Vrel_para2 = [ur_para2; vr_para2; wr_para2];
V_norm_para2 = norm([ur_para2; vr_para2; wr_para2]);

%% PARACHUTE 1 REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity
if V_norm_para1 < 1e-3
    t_vers1 = [0; 0; -1];
else
    t_vers1 = -Vrel_para1/V_norm_para1;
end

%% PARACHUTE 2 REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity
if V_norm_para2 < 1e-3
    t_vers2 = [0; 0; -1];
else
    t_vers2 = -Vrel_para2/V_norm_para2;
end

%% ATMOSPHERE DATA
% since z_rocket is similar to z_para, atmospherical data will be computed
% on z_rocket
absoluteAltitude = -z_rocket + settings.z0;
[~, a, P, rho] = atmosisa(absoluteAltitude);
M = V_norm_rocket/a;
M_value = M;

%% RELATIVE POSITION AND VELOCITY VECTORS (PARA1)
% (NED) positions of parachutes and rocket
posPara1 = [x_para1; y_para1; z_para1];
posPara2 = [x_para2; y_para2; z_para2];
posRocket = [x_rocket; y_rocket; z_rocket];
xcg = settings.xcg(2);

% (NED) position of the point from where the parachute is deployed
posDepl = posRocket + dcm'*[(xcg-settings.Lnose); 0; 0];

% (NED) relative position between parachutes and rocket
posRel1 = posPara1 - posDepl;
posRel2 = posPara2 - posDepl;

if norm(posRel1) < 1e-3
    posRel1_vers = [0; 0; -1];
else
    posRel1_vers = posRel1/norm(posRel1);
end

if norm(posRel2) < 1e-3
    posRel2_vers = [0; 0; -1];
else
    posRel2_vers = posRel2/norm(posRel2);
end

% (NED) velocities of parachutes and rocket
velPara1 = [u_para1; v_para1; w_para1];
velPara2 = [u_para2; v_para2; w_para2];
velRocket = dcm'*[u_rocket; v_rocket; w_rocket];

% (NED) velocity of the point from where the parachute is deployed
velDepl = velRocket + dcm'*cross([p; q; r],[(xcg-settings.Lnose); 0; 0]);

% (NED) relative velocity between parachutes and rocket
velRel1 = velPara1 - velDepl;
velRel2 = velPara2 - velDepl;

% Relative velocity projected along the shock chord
velRel1_chord = dot(velRel1,posRel1_vers);
velRel2_chord = dot(velRel2,posRel2_vers);

%% CHORD TENSION (ELASTIC-DAMPING MODEL) PARA1
if norm(posRel1) > (settings.para(para(2)).L + settings.para(para(1)).L)                  
    T_chord1 = (norm(posRel1) - (settings.para(para(2)).L + settings.para(para(1)).L))* settings.para(para(1)).K +...
        velRel1_chord * settings.para(para(1)).C;
else
    T_chord1 = 0;
end

%% CHORD TENSION (ELASTIC-DAMPING MODEL) PARA2
if norm(posRel2) > settings.para(para(2)).L                
    T_chord2 = (norm(posRel2) - settings.para(para(2)).L)* settings.para(para(2)).K +...
        velRel2_chord * settings.para(para(2)).C;
else
    T_chord2 = 0;
end

%% PARACHUTES FORCES PARA 1
% computed in the NED-frame reference system
t0 = settings.para(para(1)).nf * D0_1/V_norm_para1;
tx = t0 * settings.para(para(1)).CX^(1/settings.para(para(1)).m);

dt = t-t0p(1);

if dt < 0
    SCD_para1 = 0;
elseif dt < tx
    SCD_para1 = SCD0_1 * (dt/t0)^settings.para(para(1)).m;
else
    SCD_para1 = SCD0_1 * (1+(settings.para(para(1)).CX-1)*exp(-2*(dt-tx)/t0));
end

D_para1 = 0.5*rho*V_norm_para1^2*SCD_para1*t_vers1;
Fg_para1 = [0; 0; m_para1*g];                                

Ft_chord_para1 = -T_chord1 * posRel1_vers;

F_para1 = D_para1 + Fg_para1 + Ft_chord_para1;           

%% PARACHUTE FORCES PARA 2
% computed in the NED-frame reference system
t0 = settings.para(para(2)).nf * D0_2/V_norm_para2;
tx = t0 * settings.para(para(2)).CX^(1/settings.para(para(2)).m);

dt = t-t0p(2);

if dt < 0
    SCD_para2 = 0;
elseif dt < tx
    SCD_para2 = SCD0_2 * (dt/t0)^settings.para(para(2)).m;
else
    SCD_para2 = SCD0_2 * (1+(settings.para(para(2)).CX-1)*exp(-2*(dt-tx)/t0));
end

D_para2 = 0.5*rho*V_norm_para2^2*SCD_para2*t_vers2;
Fg_para2 = [0; 0; m_para2*g];                                 

Ft_chord_para2 = -T_chord2 * posRel2_vers;                     
F_para2 = D_para2 + Fg_para2 + Ft_chord_para2;               

%% ROCKET FORCES
% computed in the body-frame reference system
Fg_rocket = dcm*[0; 0; m_rocket*g];               

Ft_chord_rocket1 = T_chord1 * dcm*posRel1_vers;          
Ft_chord_rocket2 = T_chord2 * dcm*posRel2_vers;
F_rocket = Fg_rocket + Ft_chord_rocket1 + Ft_chord_rocket2;       

%% ROCKET STATE DERIVATIVES
% velocity (BODY frame)
du_rocket = F_rocket(1)/m_rocket-q*w_rocket+r*v_rocket;
dv_rocket = F_rocket(2)/m_rocket-r*u_rocket+p*w_rocket;
dw_rocket = F_rocket(3)/m_rocket-p*v_rocket+q*u_rocket;

% Rotation
b = [(xcg-settings.Lnose) 0 0];
Momentum = cross(b, (Ft_chord_rocket1+Ft_chord_rocket2));       

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
% para 1
du_para1 = F_para1(1)/m_para1;
dv_para1 = F_para1(2)/m_para1;
dw_para1 = F_para1(3)/m_para1;

% para 2
du_para2 = F_para2(1)/m_para2;
dv_para2 = F_para2(2)/m_para2;
dw_para2 = F_para2(3)/m_para2;

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
dY(17:19) = Vels_para1;
dY(20) = du_para1;
dY(21) = dv_para1;
dY(22) = dw_para1;
dY(23:25) = Vels_para2;
dY(26) = du_para2;
dY(27) = dv_para2;
dY(28) = dw_para2;
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

parout.forces.T_chord = [T_chord1, T_chord2];
parout.forces.D = [norm(D_para1), norm(D_para2)];

parout.SCD = [SCD_para1/SCD0_1, SCD_para2/SCD0_2];