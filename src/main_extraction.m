function [dY, parout] = main_extraction(t, Y, settings, uw, vw, ww, para1, para2, t0p, uncert, Hour, Day)
%% RECALLING THE STATE
% Rocket state
x_rocket = Y(1);
y_rocket = Y(2);
z_rocket = Y(3);
u_rocket = Y(4);
v_rocket = Y(5);
w_rocket = Y(6);
p_rocket = Y(7);
q_rocket = Y(8);
r_rocket = Y(9);
q0_rocket = Y(10);
q1_rocket = Y(11);
q2_rocket = Y(12);
q3_rocket = Y(13);
m_rocket = settings.ms;
Ixx = settings.Ixxe;
Iyy = settings.Iyye;
Izz = settings.Izze;

Q_rocket = [ q0_rocket q1_rocket q2_rocket q3_rocket];
Q_conj_rocket = [ q0_rocket -q1_rocket -q2_rocket -q3_rocket];
normQ_rocket = norm(Q_rocket);

if abs(normQ_rocket-1) > 0.1
    Q_rocket = Q_rocket/normQ_rocket;
end

% first parachute state
x_para1 = Y(14);
y_para1 = Y(15);
z_para1 = Y(16);
u_para1 = Y(17);
v_para1 = Y(18);
w_para1 = Y(19);

m_para1 = settings.mnc + para1.mass;

% second parachute state
x_para2 = Y(20);
y_para2 = Y(21);
z_para2 = Y(22);
u_para2 = Y(23);
v_para2 = Y(24);
w_para2 = Y(25);

m_para2 = para2.mass;

%% ADDING WIND (supposed to be added in NED axes);
if settings.wind.model
    if settings.stoch.N > 1
        [uw,vw,ww] = wind_matlab_generator(settings,z_rocket,t,Hour,Day);
    else
        [uw,vw,ww] = wind_matlab_generator(settings,z_rocket,t);
    end 
elseif settings.wind.input
    [uw,vw,ww] = wind_input_generator(settings,z_rocket,uncert);
end

wind = quatrotate(Q_rocket,[uw vw ww]);

% Rocket (BODY) relative velocities (plus wind);
ur_rocket = u_rocket - wind(1);
vr_rocket = v_rocket - wind(2);
wr_rocket = w_rocket - wind(3);

Vels_rocket = quatrotate(Q_conj_rocket,[u_rocket v_rocket w_rocket]);
V_norm_rocket = norm([ur_rocket vr_rocket wr_rocket]);

% Parachute 1 (NED) relative velocities (plus wind) 
ur_para1 = u_para1 - uw;
vr_para1 = v_para1 - vw;
wr_para1 = w_para1 - ww;

Vels_para1 = [u_para1 v_para1 w_para1];
V_norm_para1 = norm([ur_para1 vr_para1 wr_para1]);

% Parachute 2 (NED) relative velocities (plus wind) 
ur_para2 = u_para2 - uw;
vr_para2 = v_para2 - vw;
wr_para2 = w_para2 - ww;

Vels_para2 = [u_para2 v_para2 w_para2];
V_norm_para2 = norm([ur_para2 vr_para2 wr_para2]);

%% PARACHUTE REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity

t_vect = -[ur_para1 vr_para1 wr_para1];                                          % Tangenzial vector
h_vect = [vr_para1 -ur_para1 0];                                                % horizontal vector    

if all(abs(h_vect) < 1e-8)
    if all([uw vw 0] == 0) % to prevent NaN
        t_vers = t_vect/norm(t_vect);                                         % Tangenzial versor
        h_vers = [0 0 0];                                                     % horizontal versor
        n_vers = [0 0 0];                                                     % Normal versor
    else
        h_vect = [vw -uw 0];
        t_vers = t_vect/norm(t_vect);
        h_vers = h_vect/norm(h_vect);
        n_vect = cross(t_vers, h_vers);
        n_vers = n_vect/norm(n_vect);
    end
else
    t_vers = t_vect/norm(t_vect);
    h_vers = h_vect/norm(h_vect);
    n_vect = cross(t_vers, h_vers);
    n_vers = n_vect/norm(n_vect);
end

if (n_vers(3) > 0) % If the normal vector is downward directed
    n_vect = cross(h_vers, t_vers);
    n_vers = n_vect/norm(n_vect);
end

%% CONSTANTS
% Everything related to empty condition (descent-fase)
g = 9.80655;                                                                  % [N/kg] module of gravitational field at zero
T = 0;                                                                        % No Thrust

%% ATMOSPHERE DATA
% since z_rocket is similar to z_para, atmospherical data will be computed
% on z_rocket
[~, a, P, rho] = atmosisa(-z_rocket+settings.z0);
M_rocket = V_norm_rocket/a;
M_value_rocket = M_rocket;

%% RELATIVE POSITION AND VELOCITY VECTORS
% (NED) positions of parachutes
pos_para1 = [x_para1 y_para1 z_para1]; pos_para2 = [x_para2 y_para2 z_para2];

% (NED) parachute velocities
vel_para1 = [u_para1 v_para1 w_para1]; vel_para2 = [u_para2 v_para2 w_para2];

% (NED) relative postion vector, pointed towards parachute 1
relPos_vecNED = pos_para1 - pos_para2;

% (NED) relative velocity vector
relVel_vecNED = vel_para1 - vel_para2;

relPos_versNED = relPos_vecNED/norm(relPos_vecNED);

if all(abs(relPos_vecNED) < 1e-8)          % to prevent NaN
    relPos_vecNED = [0 0 0];
    relPos_versNED = [0 0 0];
end

% if > 0 the two parachutes are getting away from each other
relVel_chord = relVel_vecNED * relPos_versNED';

%% CHORD TENSION (ELASTIC-DAMPING MODEL)
if norm(relPos_vecNED) > (para2.L - para1.L)                      % [N] Chord tension (elastic-damping model)
    T_chord = (norm(relPos_vecNED) - (para2.L - para1.L))* para1.K +...
        relVel_chord * para1.C;
else
    T_chord = 0;
end

%% PARACHUTES FORCES
% computed in the NED-frame reference system
S_para1 = para1.S;                                                   % [m^2]   Surface
CD_para1 = para1.CD;

D_para1 = 0.5*rho*V_norm_para1^2*S_para1*CD_para1*t_vers';

Fg_para1 = [0 0 m_para1*g]';                                                    % [N] Gravitational Force vector
Fg_para2 = [0 0 m_para2*g]';

Ft_chord_para1 = -T_chord * relPos_versNED;                                    % [N] Chord tension vector
Ft_chord_para2 = T_chord * relPos_versNED;

F_para1 = D_para1 + Fg_para1 + Ft_chord_para1';                          % [N] (BODY) total forces vector
F_para2 = Fg_para2 + Ft_chord_para2';


%% ROCKET FORCES
% computed in the body-frame reference system
Fg_rocket = quatrotate(Q_rocket,[0 0 m_rocket*g])';                           % [N] force due to the gravity

F_rocket = Fg_rocket;                                      % [N] (NED) total forces vector

%% ROCKET STATE DERIVATIVES
% velocity (BODY frame)
du_rocket = F_rocket(1)/m_rocket-q_rocket*w_rocket+r_rocket*v_rocket;
dv_rocket = F_rocket(2)/m_rocket-r_rocket*u_rocket+p_rocket*w_rocket;
dw_rocket = F_rocket(3)/m_rocket-p_rocket*v_rocket+q_rocket*u_rocket;

% Rotation
dp_rocket = (Iyy-Izz)/Ixx*q_rocket*r_rocket;
dq_rocket = (Izz-Ixx)/Iyy*p_rocket*r_rocket;
dr_rocket = (Ixx-Iyy)/Izz*p_rocket*q_rocket;

% Quaternion
OM = 1/2* [ 0 -p_rocket -q_rocket -r_rocket  ;
            p_rocket  0  r_rocket -q_rocket  ;
            q_rocket -r_rocket  0  p_rocket  ;
            r_rocket  q_rocket -p_rocket  0 ];

dQQ_rocket = OM*Q_rocket';

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
dY(10:13) = dQQ_rocket;
dY(14:16) = Vels_para1;
dY(17) = du_para1;
dY(18) = dv_para1;
dY(19) = dw_para1;
dY(20:22) = Vels_para2;
dY(23) = du_para2;
dY(24) = dv_para2;
dY(25) = dw_para2;
dY(26:28) = [p_rocket q_rocket r_rocket];
dY=dY';

%% SAVING THE QUANTITIES FOR THE PLOTS
% Drogue data
parout.integration.t = t;

parout.interp.M = M_value_rocket;
parout.interp.alt = -z_rocket;
parout.interp.alpha = 0;
parout.interp.beta = 0;

parout.wind.NED_wind = [uw, vw, ww];
parout.wind.body_wind = wind;

parout.velocities = Vels_rocket;

parout.forces.T = T;
parout.forces.T_chord = T_chord;

parout.SCD = S_para1*CD_para1;

parout.air.rho = rho;
parout.air.P = P;

parout.accelerations.body_acc = [du_rocket, dv_rocket, dw_rocket];
parout.accelerations.ang_acc = [dp_rocket, dq_rocket, dr_rocket];

% Main data
parout.integration.t2 = t;

parout.interp.M2 = M_value_rocket;
parout.interp.alt2 = -z_rocket;
parout.interp.alpha2 = 0;
parout.interp.beta2 = 0;

parout.wind.NED_wind2 = [uw, vw, ww];
parout.wind.body_wind2 = wind;

parout.velocities2 = Vels_rocket;

parout.forces.T2 = T;
parout.forces.T_chord2 = T_chord;

parout.SCD2 = 0;

parout.air.rho2 = rho;
parout.air.P2 = P;

parout.accelerations.body_acc2 = [du_rocket, dv_rocket, dw_rocket];
parout.accelerations.ang_acc2 = [dp_rocket, dq_rocket, dr_rocket];