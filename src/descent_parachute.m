function [dY, parout] = descent_parachute(t, Y, settings, uw, vw, ww, para, t0p, uncert, Hour, Day)
%{ 

DESCENT_PARACHUTE - ode function of the 6DOF Rigid Rocket Model

INPUTS:      
            - t, integration time;
            - Y(1:17), rocket state vector, [ x y z | u v w | p q r | q0 q1 q2 q3 | m | Ixx Iyy Izz ]:

                                * (x y z), NED{north, east, down} horizontal frame; 
                                * (u v w), body frame velocities;
                                * (p q r), body frame angular rates;
                                *  m , total mass;
                                * (Ixx Iyy Izz), Inertias;
                                * (q0 q1 q2 q3), attitude unit quaternion.

            - Y(18:34), parachute sate vector, [ x y z | u v w ]:

                                * (x y z), NED{north, east, down} horizontal frame; 
                                * (u v w), body frame velocities.

            - settings, rocket data structure;
            - uw, wind component along x;
            - vw, wind component along y;
            - ww, wind component along z;
            - uncert, wind uncertanties;
            - Hour, hour of the day of the needed simulation;
            - Day, day of the month of the needed simulation;

OUTPUTS:    
            - dY, state derivatives;
            - parout, interesting fligth quantities structure (aerodyn coefficients, forces and so on..).

Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu
April 2014; Last revision: 31.XII.2014

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu
Release date: 16/04/2016

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 13/01/2018

%}

% recalling the state

% ROCKET STATE
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

% PARACHUTE STATE
x_para = Y(14);
y_para = Y(15);
z_para = Y(16);
u_para = Y(17);
v_para = Y(18);
w_para = Y(19);
m_para = settings.mnc + settings.para(para).mass;


%% ADDING WIND (supposed to be added in NED axes);
if settings.wind.model
    
    if settings.stoch.N > 1
        [uw,vw,ww] = wind_matlab_generator(settings,z,t,Hour,Day);
    else
        [uw,vw,ww] = wind_matlab_generator(settings,z,t);
    end
    
elseif settings.wind.input
    
    [uw,vw,ww] = wind_input_generator(settings,z,uncert);
    
end

wind = quatrotate(Q_rocket,[uw vw ww]);

% Rocket relative velocities (plus wind);
ur_rocket = u_rocket - wind(1);
vr_rocket = v_rocket - wind(2);
wr_rocket = w_rocket - wind(3);

Vels_rocket = quatrotate(Q_conj_rocket,[u_rocket v_rocket w_rocket]);
V_norm_rocket = norm([ur_rocket vr_rocket wr_rocket]);

% Parachute relative velocities (plus wind)
ur_para = u_para - wind(1);
vr_para = v_para - wind(2);
wr_para = w_para - wind(3);

Vels_para = quatrotate(Q_conj_rocket,[u_para v_para w_para]);
V_norm_para = norm([ur_para vr_para wr_para]);

%% PARACHUTE REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity

t_vect = -[ur_para vr_para wr_para];                % Tangenzial vector
h_vect = [vr_para -ur_para 0];                     % horizontal vector    

if all(abs(h_vect) < 1e-8)
    h_vect = [vw -uw 0];
end

t_vers = t_vect/norm(t_vect);            % Tangenzial versor
h_vers = h_vect/norm(h_vect);            % horizontal versor

n_vect = cross(t_vers, h_vers);          % Normal vector
n_vers = n_vect/norm(n_vect);            % Normal versor

if (n_vers(3) > 0)                       % If the normal vector is downward directed
    n_vect = cross(h_vers, t_vers);
    n_vers = n_vect/norm(n_vect);
end

%% PARACHUTE CONSTANTS
% CD and S will be computed later
CL_para = settings.para(para).CL;                                             % [/] Parachute Lift Coefficient

%% ROCKET CONSTANTS
% Everything related to empty condition (descent-fase)

S_rocket = settings.S;                          % [m^2] cross surface
C_rocket = settings.C;                          % [m]   caliber
CoeffsE = settings.CoeffsE;                     % [/] Empty Rocket Coefficients
CoeffsDesc = settings.CoeffsDesc;               % [/]
g = 9.80655;                                    % [N/kg] module of gravitational field at zero
T = 0;                                          % No Thrust :)

%% ATMOSPHERE DATA
% since z_rocket is similar to z_para, atmospherical data will be computed
% on z_rocket

if -z_rocket < 0     % z is directed as the gravity vector
    z_rocket = 0;
end

[~, a, P, rho] = atmosisa(-z_rocket+settings.z0);
M_rocket = V_norm_rocket/a;
M_value_rocket = M_rocket;

%% AERODYNAMICS ANGLES
if not(ur_rocket < 1e-9 || V_norm_rocket < 1e-9)
    alpha = atan(wr_rocket/ur_rocket);
    beta = atan(vr_rocket/ur_rocket);             % beta = asin(vr/V_norm); is the classical notation, Datcom uses this one though. 
else
    alpha = 0;
    beta = 0;
end

alpha_value = alpha;
beta_value = beta;


%% DATCOM COEFFICIENTS

givA = settings.Alphas*pi/180;
givB = settings.Betas*pi/180;
givH = settings.Altitudes;
givM = settings.Machs;

%% INTERPOLATION AT THE BOUNDARIES

if M_rocket > givM(end)
    M_rocket = givM(end);
elseif M_rocket < givM(1)
    M_rocket = givM(1);
end

if alpha > givA(end)
    alpha = givA(end);
elseif alpha < givA(1)
    alpha = givA(1);
end

if beta > givB(end)
    beta = givB(end);
elseif beta < givB(1)
    beta = givB(1);
end

if -z_rocket > givH(end)
    z_rocket = -givH(end);
elseif -z_rocket < givH(1)
    z_rocket = -givH(1);
end

%% CHOSING THE CONDITION VALUE
% interpolation of the coefficients with the value in the nearest condition of the Coeffs matrix

c = 1; % descent with no aerobrakes

[CA, angle0] = interp4_easy(givA,givM,givB,givH,CoeffsE.CA(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
CYB = interp4_easy(givA,givM,givB,givH,CoeffsE.CYB(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
CY0 = interp4_easy(givA,givM,givB,givH,CoeffsE.CY(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
CNA = interp4_easy(givA,givM,givB,givH,CoeffsE.CNA(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
CN0 = interp4_easy(givA,givM,givB,givH,CoeffsE.CN(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cl = interp4_easy(givA,givM,givB,givH,CoeffsE.CLL(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Clp = interp4_easy(givA,givM,givB,givH,CoeffsE.CLLP(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cma = interp4_easy(givA,givM,givB,givH,CoeffsE.CMA(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cm0 = interp4_easy(givA,givM,givB,givH,CoeffsE.CM(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cmad = interp4_easy(givA,givM,givB,givH,CoeffsE.CMAD(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cmq = interp4_easy(givA,givM,givB,givH,CoeffsE.CMQ(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cnb = interp4_easy(givA,givM,givB,givH,CoeffsE.CLNB(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cn0 = interp4_easy(givA,givM,givB,givH,CoeffsE.CLN(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cnr = interp4_easy(givA,givM,givB,givH,CoeffsE.CLNR(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);
Cnp = interp4_easy(givA,givM,givB,givH,CoeffsE.CLNP(:, :, :, :, c),alpha,M_rocket,beta,-z_rocket);

% compute CN,CY,Cm,Cn (linearized with respect to alpha and beta):
alpha0 = angle0(1); beta0 = angle0(2);

CN = (CN0 + CNA*(alpha-alpha0));
CY = (CY0 + CYB*(beta-beta0));
Cm = (Cm0 + Cma*(alpha-alpha0));
Cn = (Cn0 + Cnb*(beta-beta0));


%% RELATIVE POSITION AND VELOCITY VECTORS
% NED positions of parachute and rocket
pos_para = [x_para y_para z_para]; pos_rocket = [x_rocket y_rocket z_rocket];

% Body velocities of parachute and rocket
vel_para = [u_para v_para w_para]; vel_rocket = [u_rocket v_rocket w_rocket];

% NED relative position vector pointed from xcg to the point from where the
% parachute has been deployed
posRelXcg_Poi = quatrotate(Q_conj_rocket,[(settings.xcg(2)-settings.Lnc) 0 0]);

% NED position vector of that point from the origin
pos_Poi = pos_rocket + posRelXcg_Poi;

% Body velocity vector of that point
vel_Poi = vel_rocket + cross([p_rocket q_rocket r_rocket],[(settings.xcg(2)-settings.Lnc) 0 0]);

% NED Relative position vector between parachute and that point. Pointed
% towards parachute
relPos_vec = pos_para - (pos_rocket+quatrotate(Q_conj_rocket,[(settings.xcg(2)-settings.Lnc) 0 0]));
relPos_vec = quatrotate(Q_rocket,relPos_vec);
relPos_vers = relPos_vec/norm(relPos_vec);

if all(relPos_vec < 1e-8)
    relPos_vec = [0 0 0];
    relPos_vers = [0 0 0];
end

% Body relative velocity vector between parachute and that point
relVel_vec = vel_Poi - vel_para;

% Body relative velocity projected along the chock chord. Pointed towards
% parachute
relVel_chord = relVel_vec * relPos_vers';


%% PARACHUTE FORCES
S_para = settings.para(para).S;                                           % [m^2]   Surface
if t < t0p + settings.para(para).OverExp_t
    SCD_para = (settings.para(para).S*settings.para(para).CD)/...             % Linear interpolation for the expansion phase
        (settings.para(para).OverExp_t) * t;
    D_para = 0.5*rho*V_norm_para^2*SCD_para*t_vers';                          % [N] Drag vector
else
    CD_para = settings.para(para).CD;                                         % [/] Parachute Drag Coefficient
    D_para = 0.5*rho*V_norm_para^2*S_para*CD_para*t_vers';                    % [N] Drag vector
end

L_para = 0.5*rho*V_norm_para^2*S_para*CL_para*n_vers';       % [N] Lift vector
Fg_para = quatrotate(Q_rocket,[0 0 m_para*g])';                                 % [N] Gravitational Force vector

if norm(relPos_vec) > settings.para(para).ShockCord_L
    T_chord = (norm(relPos_vec) - settings.para(para).ShockCord_L)*settings.para(1).ShockCord_k...
        - relVel_chord * settings.para(para).ShockCord_c;
else
    T_chord = 0;
end

Ft_chord_para = -T_chord * relPos_vers;                              % [N] chord tension vector
F_para = D_para + Ft_chord_para' + L_para + Fg_para;                  % [N] (BODY) total forces vector in NED frame

%% ROCKET FORCES
% first computed in the body-frame reference system
qdyn = 0.5*rho*V_norm_rocket^2;        % [Pa] dynamics pressure
qdynL_V = 0.5*rho*V_norm_rocket*S_rocket*C_rocket;   %

X = qdyn*S_rocket*CA;                                % [N] x-body component of the aerodynamics force
Y = qdyn*S_rocket*CY;                                % [N] y-body component of the aerodynamics force
Z = qdyn*S_rocket*CN;                                % [N] z-body component of the aerodynamics force
Fg_rocket = quatrotate(Q_rocket,[0 0 m_rocket*g])';  % [N] force due to the gravity

Ft_chord_rocket = T_chord * relPos_vers;                    % [N] Chord Tension in Rocket frame
F_rocket = Fg_rocket + Ft_chord_rocket' + [-X,+Y,-Z]';      % [N] total forces vector in body frame

quatrotate(Q_conj_rocket,F_rocket')'
%% ROCKET STATE DERIVATIVES
b = [(settings.xcg(2)-settings.Lnc) 0 0];
% velocity
du_rocket = F_rocket(1)/m_rocket-q_rocket*w_rocket+r_rocket*v_rocket;
dv_rocket = F_rocket(2)/m_rocket-r_rocket*u_rocket+p_rocket*w_rocket;
dw_rocket = F_rocket(3)/m_rocket-p_rocket*v_rocket+q_rocket*u_rocket;

% Rotation
Momentum = cross(b, Ft_chord_rocket);

dp_rocket = (Iyy-Izz)/Ixx*q_rocket*r_rocket + Momentum(1) + qdynL_V/Ixx*(V_norm_rocket*Cl+Clp*p_rocket*C_rocket/2);
dq_rocket = (Izz-Ixx)/Iyy*p_rocket*r_rocket + Momentum(2) + qdynL_V/Iyy*(V_norm_rocket*Cm + (Cmad+Cmq)*q_rocket*C_rocket/2);
dr_rocket = (Ixx-Iyy)/Izz*p_rocket*q_rocket + Momentum(3) + qdynL_V/Izz*(V_norm_rocket*Cn + (Cnr*r_rocket+Cnp*p_rocket)*C_rocket/2);

% Quaternion
OM = 1/2* [ 0 -p_rocket -q_rocket -r_rocket  ;
            p_rocket  0  r_rocket -q_rocket  ;
            q_rocket -r_rocket  0  p_rocket  ;
            r_rocket  q_rocket -p_rocket  0 ];

dQQ_rocket = OM*Q_rocket';

%% PARACHUTE STATE DERIVATIVES
% velocity
du_para = F_para(1)/m_para-q_rocket*w_para+r_rocket*v_para;
dv_para = F_para(2)/m_para-r_rocket*u_para+p_rocket*w_para;
dw_para = F_para(3)/m_para-p_rocket*v_para+q_rocket*u_para;

%% FINAL DERIVATIVE STATE ASSEMBLING
% Rocket
dY(1:3) = Vels_rocket;
dY(4) = du_rocket;
dY(5) = dv_rocket;
dY(6) = dw_rocket;
dY(7) = dp_rocket;
dY(8) = dq_rocket;
dY(9) = dr_rocket;
dY(10:13) = dQQ_rocket;
dY(14:16) = Vels_para;
dY(17) = du_para;
dY(18) = dv_para;
dY(19) = dw_para;
dY=dY';

parout.integration.t = t;
parout.interp.alt = -z_rocket;
parout.wind.body_wind = wind;
parout.wind.NED_wind = [uw, vw, ww];

parout.air.rho = rho;
parout.air.P = P;

parout.accelerations.body_acc = [du_rocket, dv_rocket, dw_rocket];

parout.velocities = [u_rocket, v_rocket, w_rocket];


% function [dY, parout] = descent_parachute(t, Y, settings, uw, vw, ww, para, uncert, Hour, Day)
% %{ 
% 
% ASCENT - ode function of the 6DOF Rigid Rocket Model
% 
% INPUTS:      
%             - t, integration time;
%             - Y, state vector, [ x y z | u v w ]:
% 
%                                 * (x y z), NED{north, east, down} horizontal frame; 
%                                 * (u v w), horizontal frame velocities.
% 
%             - settings, rocket data structure;
%             - uw, wind component along x;
%             - vw, wind component along y;
%             - ww, wind component along z;
%             - uncert, wind uncertanties;
%             - Hour, hour of the day of the needed simulation;
%             - Day, day of the month of the needed simulation;
% 
% OUTPUTS:    
%             - dY, state derivatives;
%             - parout, interesting fligth quantities structure (aerodyn coefficients, forces and so on..).
% 
% Author: Ruben Di Battista
% Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
% email: ruben.dibattista@skywarder.eu
% April 2014; Last revision: 31.XII.2014
% 
% Author: Francesco Colombi
% Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
% email: francesco.colombi@skywarder.eu
% Release date: 16/04/2016
% 
% Author: Adriano Filippo Inno
% Skyward Experimental Rocketry | AFD Dept | crd@skywarder.eu
% email: adriano.filippo.inno@skywarder.eu
% Release date: 13/01/2018
% 
% %}
% 
% % recalling the state
% % x = Y(1);
% % y = Y(2);
% z = Y(3);
% u = Y(4);
% v = Y(5);
% w = Y(6);
% 
% %% ADDING WIND (supposed to be added in NED axes);
% 
% if settings.wind.model
%     
%     if settings.stoch.N > 1
%         [uw,vw,ww] = wind_matlab_generator(settings,z,t,Hour,Day);
%     else
%         [uw,vw,ww] = wind_matlab_generator(settings,z,t);
%     end
%     
% elseif settings.wind.input
%     
%     [uw,vw,ww] = wind_input_generator(settings,z,uncert);
%     
% end
% 
% wind = [uw vw ww];
% 
% % Relative velocities (plus wind);
% ur = u - wind(1);
% vr = v - wind(2);
% wr = w - wind(3);
% 
% V_norm = norm([ur vr wr]);
% 
% %% CONSTANTS
% S = settings.para(para).S;                                               % [m^2]   Surface
% CD = settings.para(para).CD;                                             % [/] Parachute Drag Coefficient
% CL = settings.para(para).CL;                                             % [/] Parachute Lift Coefficient
% if para == 1
%     pmass = 0 ;                                                          % [kg] detached mass
% else
%     pmass = sum(settings.para(1:para-1).mass) + settings.mnc;
% end
% 
% g = 9.80655;                                                             % [N/kg] magnitude of the gravitational field at zero
% m = settings.ms - pmass;                                                 % [kg] descend mass
% 
% %% ATMOSPHERE DATA
% 
% if -z < 0
%     z = 0;
% end
% 
% [~, ~, P, rho] = atmosisa(-z+settings.z0);
% 
% 
% %% REFERENCE FRAME
% % The parachutes are approximated as rectangular surfaces with the normal
% % vector perpendicular to the relative velocity
% 
% t_vect = [ur vr wr];                     % Tangenzial vector
% h_vect = [-vr ur 0];                     % horizontal vector    
% 
% if all(abs(h_vect) < 1e-8)
%     h_vect = [-vw uw 0];
% end
% 
% t_vers = t_vect/norm(t_vect);            % Tangenzial versor
% h_vers = -h_vect/norm(h_vect);           % horizontal versor
% 
% n_vect = cross(t_vers, h_vers);          % Normal vector
% n_vers = n_vect/norm(n_vect);            % Normal versor
% 
% if (n_vers(3) > 0)                       % If the normal vector is downward directed
%     n_vect = cross(h_vers, t_vers);
%     n_vers = n_vect/norm(n_vect);
% end
% 
% %% FORCES
% 
% D = 0.5*rho*V_norm^2*S*CD*t_vers';       % [N] Drag vector
% L = 0.5*rho*V_norm^2*S*CL*n_vers';       % [N] Lift vector
% Fg = m*g*[0 0 1]';                       % [N] Gravitational Force vector
% F = -D+L+Fg;                             % [N] total forces vector
% 
% %% STATE DERIVATIVES
% 
% % velocity
% du = F(1)/m;
% dv = F(2)/m;
% dw = F(3)/m;
% 
% %% FINAL DERIVATIVE STATE ASSEMBLING
% 
% dY(1:3) = [u v w]';
% dY(4) = du;
% dY(5) = dv;
% dY(6) = dw;
% 
% dY = dY';
% 
% %% SAVING THE QUANTITIES FOR THE PLOTS
% 
% if settings.plots
%     
%     parout.integration.t = t;
%     parout.interp.alt = -z;
%     parout.wind.body_wind = [uw, vw, ww];
%     parout.wind.NED_wind = [uw, vw, ww];
%     
%     parout.air.rho = rho;
%     parout.air.P = P;
%     
%     parout.accelerations.body_acc = [du, dv, dw];
%     
%     parout.velocities = [u, v, w];
%     
% end