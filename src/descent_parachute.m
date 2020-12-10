function [dY, parout] = descent_parachute(t, Y, settings, t0p, uw, vw, ww, para, uncert, Hour, Day)
%{ 

ASCENT - ode function of the 6DOF Rigid Rocket Model

INPUTS:      
            - t, integration time;
            - Y(1:17), rocekt state vector, [ x y z | u v w | p q r | q0 q1 q2 q3 | m | Ixx Iyy Izz ]
            - Y(18:34), parachute sate vector, [ x y z | u v w | p q r | q0 q1 q2 q3 | m | Ixx Iyy Izz ]:

                                * (x y z), NED{north, east, down} horizontal frame; 
                                * (u v w), body frame velocities;
                                * (p q r), body frame angular rates;
                                *  m , total mass;
                                * (Ixx Iyy Izz), Inertias;
                                * (q0 q1 q2 q3), attitude unit quaternion.

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
m_rocket = Y(14);
Ixx_rocket = Y(15);
Iyy_rocket = Y(16);
Izz_rocket = Y(17);

Q_rocket = [ q0_rocket q1_rocket q2_rocket q3_rocket];
Q_conj_rocket = [ q0_rocket -q1_rocket -q2_rocket -q3_rocket];
normQ_rocket = norm(Q_rocket);

Q_rocket = Q_rocket/normQ_rocket;

% PARACHUTE STATE
x_para = Y(18);
y_para = Y(19);
z_para = Y(20);
u_para = Y(21);
v_para = Y(22);
w_para = Y(23);
p_para = Y(24);
q_para = Y(25);
r_para = Y(26);
q0_para = Y(27);
q1_para = Y(28);
q2_para = Y(29);
q3_para = Y(30);
m_para = Y(31);
Ixx_para = Y(32);
Iyy_para = Y(33);
Izz_para = Y(34);

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

wind = [uw vw ww];

% Rocket relative velocities (plus wind);
ur_rocket = u_rocket - wind(1);
vr_rocket = v_rocket - wind(2);
wr_rocket = w_rocket - wind(3);

V_norm_rocket = norm([ur_rocket vr_rocket wr_rocket]);

% Parachute relative velocities (plus wind)
ur_para = u_para - wind(1);
vr_para = v_para - wind(2);
wr_para = w_para - wind(3);

V_norm_para = norm([ur_para vr_para wr_para]);

%% PARACHUTE REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity

t_vect = [ur_para vr_para wr_para];                % Tangenzial vector
h_vect = [-vr_para ur_para 0];                     % horizontal vector    

if all(abs(h_vect) < 1e-8)
    h_vect = [-vw uw 0];
end

t_vers = t_vect/norm(t_vect);            % Tangenzial versor
h_vers = -h_vect/norm(h_vect);           % horizontal versor

n_vect = cross(t_vers, h_vers);          % Normal vector
n_vers = n_vect/norm(n_vect);            % Normal versor

if (n_vers(3) > 0)                       % If the normal vector is downward directed
    n_vect = cross(h_vers, t_vers);
    n_vers = n_vect/norm(n_vect);
end

% position of the point from where the chord is deployed
PosChord_vec = [0 0 (z_rocket - (setting.xcg(2)-settings.Lnc)]';        % position vector in NED frame
PosChord_vec = quatrotate(Q_rocket,posChord_vec);                       % position in rocket frame  

%% PARACHUTE CONSTANTS
% CD and S will be computed later
CL_para = settings.para(para).CL;                                             % [/] Parachute Lift Coefficient

if para == 1
    m_para = 0 ;                                                               % [kg] detached mass
else
    m_para = sum(settings.para(1:para-1).mass) + settings.mnc;
end

g = 9.80655;                                                                  % [N/kg] magnitude of the gravitational field at zero

%% ROCKET CONSTANTS
% Everything related to empty condition (descent-fase)

S_rocket = settings.S;                          % [m^2] cross surface
C_rocket = settings.C;                          % [m]   caliber
CoeffsE = settings.CoeffsE;                     % [/] Empty Rocket Coefficients
g = 9.80655;                                    % [N/kg] module of gravitational field at zero
T = 0;   
m_rocket = settings.ms - m_para;                 % [kg] descend mass

%% ATMOSPHERE DATA
% since z_rocket is similar to z_para, atmospherical data will be computed
% on z_rocket

if -z < 0     % z is directed as the gravity vector
    z = 0;
end

[~, a, P, rho] = atmosisa(-z_rocket+settings.z0);
M_rocket = V_norm/a;
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

%% PARACHUTE FORCES
% first computed in the parachute-frame reference system
if t < t0p + settings.para(para).OverExp_t
    SCD_para = (settings.para(para).S*settings.para(para).CD)/...             % Linear interpolation for the expansion phase
        (settings.para(para).OverExp_t) * t;
    D_para = 0.5*rho*V_norm_para^2*SCD_para*t_vers';                          % [N] Drag vector
else
    S_para = settings.para(para).S;                                           % [m^2]   Surface
    CD_para = settings.para(para).CD;                                         % [/] Parachute Drag Coefficient
    D_para = 0.5*rho*V_norm_para^2*S_para*CD_para*t_vers';                    % [N] Drag vector
end

L_para = 0.5*rho*V_norm_para^2*S_para*CL_para*n_vers';       % [N] Lift vector
Fg_para = m_para*g*[0 0 1]';                                 % [N] Gravitational Force vector

pos_para = [x_para y_para z_para];
pos_rocket = [x_rocket y_rocket z_rocket];
vel_para = [u_para v_para w_para];
vel_rocket = [u_rocket v_rocket w_rocket]

if norm(pos_para - pos_rocket) < settings.para(para).ShockCord_L
    T_chord = (norm(pos_para-pos_rocket) - settings.para(para).ShockCord_L)*settings.para(1).ShockCord_k...
        + norm(vel_para-vel_rocket)*settings.para(para).ShockCord_c;
else
    T_chord = 0;
end

dis_vec = pos_para - pos_rocket;
dis_vers = dis_vec/norm(dis_vec);                            % relative position versor, pointed from parachute to rocket

Ft_chord_para = T_chord * dis_vers';                              % [N] chord tension vector
F_para = -D_para + Ft_chord_para + L_para + Fg_para;              % [N] total forces vector in NED frame


%% ROCKET FORCES
% first computed in the body-frame reference system
qdyn = 0.5*rho*V_norm^2;        % [Pa] dynamics pressure
qdynL_V = 0.5*rho*V_norm*S*C;   %

X = qdyn*S*CA;                  % [N] x-body component of the aerodynamics force
Y = qdyn*S*CY;                  % [N] y-body component of the aerodynamics force
Z = qdyn*S*CN;                  % [N] z-body component of the aerodynamics force
Fg = quatrotate(Q,[0 0 m*g])';  % [N] force due to the gravity

Ft_chord_rocket = quatrotate(Q, -Ft_chord_para);      % [N] Chord Tension in Rocket frame
F = Fg + Ft_chord + [-X,+Y,-Z]';                      % [N] total forces vector in body frame


