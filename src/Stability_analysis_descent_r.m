clc
close all

% data_para.m file required: run simulation before
% linearization around steady state (in this case line 424 of the drogue
% state vector)
% doesn't compute line 354 double(A)

path = genpath(pwd);
addpath(path);

%% Load data
run('config.m');

% Drogue state
T = data_para{1}.state.T;
Y = data_para{1}.state.Y;

% ROCKET
m_rocket = settings.ms;
Ixx = settings.Ixxe;
Iyy = settings.Iyye;
Izz = settings.Izze;


% Point around which linearization is evaluated: descent point line 424
X0 = [Y(424,:)]';  
t = T(424);                                            %time instant in which the linearization is evaluated

%% Init symbolic variables
syms x_rocket y_rocket z_rocket u_rocket v_rocket w_rocket p_rocket q_rocket r_rocket...
    q0_rocket q1_rocket q2_rocket q3_rocket x_para y_para z_para u_para v_para w_para ...
    theta_x theta_y theta_z real



Q_rocket = [ q0_rocket q1_rocket q2_rocket q3_rocket];
Q_conj_rocket = [ q0_rocket -q1_rocket -q2_rocket -q3_rocket];
normQ_rocket = norm(Q_rocket);

% if abs(normQ_rocket-1) > 0.1
%     Q_rocket = Q_rocket/normQ_rocket;
% end
Q_rocket = Q_rocket/normQ_rocket;

m_para = settings.mnc + sum([settings.para(1:2).mass]);

%% ADDING WIND (supposed to be added in NED axes);
if settings.wind.model || settings.wind.input   % will be computed inside the integrations
    uw = 0; vw = 0; ww = 0;
else
    [uw, vw, ww, Azw] = wind_const_generator(settings.wind.AzMin, settings.wind.AzMax,...
        settings.wind.ElMin, settings.wind.ElMax, settings.wind.MagMin, settings.wind.MagMax);
    
    if ww ~= 0
        warning('Pay attention using vertical wind, there might be computational errors')
    end
    
end
if settings.wind.model
    
    if settings.stoch.N > 1
        [uw,vw,ww] = wind_matlab_generator(settings,z,t,Hour,Day);
    else
        [uw,vw,ww] = wind_matlab_generator(settings,z,t);
    end
    
elseif settings.wind.input
    
    [uw,vw,ww] = wind_input_generator(settings,z,uncert);
    
end

qin = Q_rocket./norm(Q_rocket);

dcm(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
dcm(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
dcm(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
dcm(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
dcm(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
dcm(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
dcm(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
dcm(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
dcm(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;

wind = (dcm*[uw vw ww]')';
%wind = quatrotate(Q_rocket,[uw vw ww]);

% Rocket (BODY) relative velocities (plus wind);
ur_rocket = u_rocket - wind(1);
vr_rocket = v_rocket - wind(2);
wr_rocket = w_rocket - wind(3);

qin = Q_conj_rocket./norm(Q_conj_rocket);

dcm_conj(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
dcm_conj(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
dcm_conj(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
dcm_conj(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
dcm_conj(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
dcm_conj(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
dcm_conj(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
dcm_conj(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
dcm_conj(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;

Vels_rocket = (dcm_conj*[u_rocket v_rocket w_rocket]')'; 
%Vels_rocket = quatrotate(Q_conj_rocket,[u_rocket v_rocket w_rocket]);
V_norm_rocket = norm([ur_rocket vr_rocket wr_rocket]);

% Parachute (NED) relative velocities (plus wind) 
ur_para = u_para - uw;
vr_para = v_para - vw;
wr_para = w_para - ww;


Vels_para = [u_para v_para w_para];
V_norm_para = norm([ur_para vr_para wr_para]);

%% PARACHUTE REFERENCE FRAME
% The parachutes are approximated as rectangular surfaces with the normal
% vector perpendicular to the relative velocity

t_vect = -[ur_para vr_para wr_para];                                          % Tangenzial vector
h_vect = [vr_para -ur_para 0];                                                % horizontal vector    

% if all(abs(h_vect) < 1e-8)
%     if all([uw vw 0] == 0) % to prevent NaN
%         t_vers = t_vect/norm(t_vect);                                         % Tangenzial versor
%         h_vers = [0 0 0];                                                     % horizontal versor
%         n_vers = [0 0 0];                                                     % Normal versor
%     else
%         h_vect = [vw -uw 0];
%         t_vers = t_vect/norm(t_vect);
%         h_vers = h_vect/norm(h_vect);
%         n_vect = cross(t_vers, h_vers);
%         n_vers = n_vect/norm(n_vect);
%     end
% else
%     t_vers = [0, 0, -1];
%     n_vers = [1, 0, 0];
    t_vers = t_vect/norm(t_vect);
    h_vers = h_vect/norm(h_vect);
    n_vect = cross(t_vers, h_vers);
    n_vers = n_vect/norm(n_vect);
% end

% if (n_vers(3) > 0) % If the normal vector is downward directed
%     n_vect = cross(h_vers, t_vers);
%     n_vers = n_vect/norm(n_vect);
% end

%% PARACHUTE CONSTANTS
% CD and S will be computed later
CL_para = settings.para(1).CL;                                             % [/] Parachute Lift Coefficient

%% ROCKET CONSTANTS
% Everything related to empty condition (descent-fase)
g = 9.80655;                                                                  % [N/kg] module of gravitational field at zero
T = 0;                                                                        % No Thrust

%% ATMOSPHERE DATA
% since z_rocket is similar to z_para, atmospherical data will be computed
% on z_rocket

% if -z_rocket < 0     % z is directed as the gravity vector
%     z_rocket = 0;
% end


h = -z_rocket + settings.z0;

atmC = [9.80665, 1.4, 287.0531, 0.0065, 11000, 20000, ...
            1.225, 101325, 288.15]; % atmosisa constants:

%h(h > atmC(6)) = atmC(6);
%h(h < 0) = 0;
%hGrThHTS = (h > atmC(5));

h_tmp = h;
%h_tmp(hGrThHTS) = atmC(5);

T = atmC(9) - atmC(4)*h_tmp;

%expon = ones(size(h));
%expon(hGrThHTS) = exp(atmC(1)./(atmC(3)*T(hGrThHTS)).*(atmC(5) - h(hGrThHTS)));
expon = 1;

a = sqrt(T*atmC(2)*atmC(3));

theta = T/atmC(9);

P = atmC(8)*theta.^(atmC(1)/(atmC(4)*atmC(3))).*expon;
rho = atmC(7)*theta.^((atmC(1)/(atmC(4)*atmC(3)))-1.0).*expon;
%[~, a, P, rho] = atmosisa(-z_rocket+settings.z0);
M_rocket = V_norm_rocket/a;
M_value_rocket = M_rocket;

%% RELATIVE POSITION AND VELOCITY VECTORS
% (NED) positions of parachute and rocket
pos_para = [x_para y_para z_para]; pos_rocket = [x_rocket y_rocket z_rocket];

% (BODY) parachute velocity, (NED) rocket velocity
vel_para = [u_para v_para w_para]; vel_rocket = [u_rocket v_rocket w_rocket];

% (NED) relative position vector pointed from xcg to the point from where the
% parachute has been deployed

posRelXcg_Poi = (dcm_conj*[(settings.xcg(2)-settings.Lnc) 0 0]')';
%posRelXcg_Poi = quatrotate(Q_conj_rocket,[(settings.xcg(2)-settings.Lnc) 0 0]);

% (NED) position vector of that point from the origin
pos_Poi = pos_rocket + posRelXcg_Poi;

% (BODY) velocity vector of that point
c = (dcm*[p_rocket q_rocket r_rocket]')';
vel_Poi = vel_rocket + cross(c,[(settings.xcg(2)-settings.Lnc) 0 0]);

% Relative position vector between parachute and that point.
relPos_vecNED = pos_para - pos_Poi; % (NED) relative position vector pointed towards parachute
relPos_vecBODY = (dcm*relPos_vecNED')';
%relPos_vecBODY = quatrotate(Q_rocket,relPos_vecNED);                          % (BODY)   "         "      "       "       "       "
relPos_versNED = relPos_vecNED/norm(relPos_vecNED);                           % (NED) relative position versor pointed towards parachute
relPos_versBODY = relPos_vecBODY/norm(relPos_vecBODY);                        % (BODY)   "         "      "       "       "       "

% if all(abs(relPos_vecNED) < 1e-8) || all(abs(relPos_vecBODY) < 1e-8)          % to prevent NaN
%     relPos_vecNED = [0 0 0];
%     relPos_versNED = [0 0 0];
%     relPos_versBODY= [0 0 0];
% end

% (BODY) relative velocity vector between parachute and that point, pointed
% towards rocket
relVel_vecNED = (dcm_conj*vel_Poi')'-vel_para;
%relVel_vecNED = quatrotate(Q_conj_rocket,vel_Poi) - vel_para;

% (BODY) relative velocity projected along the chock chord. Pointed towards
% parachute
relVel_chord = relVel_vecNED * relPos_versNED';

%% CHORD TENSION (ELASTIC-DAMPING MODEL)
% if norm(relPos_vecNED) > settings.para(1).ShockCord_L                      % [N] Chord tension (elastic-damping model)
%     T_chord = (norm(relPos_vecNED) - settings.para(para).ShockCord_L)*...
%         settings.para(para).ShockCord_k - relVel_chord *...
%         settings.para(para).ShockCord_c;
% else
    T_chord = 207;
% end

%% PARACHUTE FORCES
% computed in the NED-frame reference system
S_para = settings.para(1).S;                                               % [m^2]   Surface

% if para ~= 1
%     if t < t0p(para) + settings.para(para).OverExp_t                              % Linear interpolation for the over-expansion phase
%         SCD_para = (settings.para(para).S*settings.para(para).CD)/...
%             (settings.para(para).OverExp_t) * (t-t0p(para));
%         D_para = 0.5*rho*V_norm_para^2*SCD_para*t_vers';                          % [N] Drag vector
%     else
%         CD_para = settings.para(para).CD;                                         % [/] Parachute Drag Coefficient
%         D_para = 0.5*rho*V_norm_para^2*S_para*CD_para*t_vers';                    % [N] Drag vector
%     end
%     if norm(D_para) <= norm(0.5*rho*V_norm_para^2*settings.para(para-1).S*settings.para(para-1).CD*t_vers')
%         D_para = 0.5*rho*V_norm_para^2*settings.para(para-1).S*settings.para(para-1).CD*t_vers';
%     end
% else
%     if t < t0p(1) + settings.para(1).OverExp_t                              % Linear interpolation for the over-expansion phase
%         SCD_para = (settings.para(1).S*settings.para(1).CD)/...
%             (settings.para(1).OverExp_t) * (Ta(end)-t0p(1));
%         D_para = 0.5*rho*V_norm_para^2*SCD_para*t_vers';                          % [N] Drag vector
%     else
        CD_para = settings.para(1).CD;                                         % [/] Parachute Drag Coefficient
        D_para = 0.5*rho*V_norm_para^2*S_para*CD_para*t_vers';                    % [N] Drag vector
%     end
% % end

L_para = 0.5*rho*V_norm_para^2*S_para*CL_para*n_vers';                        % [N] Lift vector
Fg_para = [0 0 m_para*g]';                                                    % [N] Gravitational Force vector

Ft_chord_para = -T_chord * relPos_versNED;                                    % [N] Chord tension vector (parachute view)
F_para = D_para + L_para + Fg_para + Ft_chord_para';                          % [N] (BODY) total forces vector

%% ROCKET FORCES
% computed in the body-frame reference system
Fg_rocket = (dcm*[0 0 m_rocket*g]')';
%Fg_rocket = quatrotate(Q_rocket,[0 0 m_rocket*g])';                           % [N] force due to the gravity

Ft_chord_rocket = T_chord * relPos_versBODY;                                  % [N] Chord tension vector (rocket view)
F_rocket = Fg_rocket + Ft_chord_rocket';                                      % [N] (NED) total forces vector

%% ROCKET STATE DERIVATIVES
% velocity (BODY frame)
du_rocket = F_rocket(1)/m_rocket-q_rocket*w_rocket+r_rocket*v_rocket;
dv_rocket = F_rocket(2)/m_rocket-r_rocket*u_rocket+p_rocket*w_rocket;
dw_rocket = F_rocket(3)/m_rocket-p_rocket*v_rocket+q_rocket*u_rocket;

% Rotation
b = (dcm_conj*[(settings.xcg(2)-settings.Lnc) 0 0]')';
%b = quatrotate(Q_conj_rocket,[(settings.xcg(2)-settings.Lnc) 0 0]);
Momentum = cross(b, -Ft_chord_para);                                          % [Nm] Chord tension moment

dp_rocket = (Iyy-Izz)/Ixx*q_rocket*r_rocket + Momentum(1)/Ixx;
dq_rocket = (Izz-Ixx)/Iyy*p_rocket*r_rocket + Momentum(2)/Iyy;
dr_rocket = (Ixx-Iyy)/Izz*p_rocket*q_rocket + Momentum(3)/Izz;

% Quaternion
OM = 1/2* [ 0 -p_rocket -q_rocket -r_rocket  ;
            p_rocket  0  r_rocket -q_rocket  ;
            q_rocket -r_rocket  0  p_rocket  ;
            r_rocket  q_rocket -p_rocket  0 ];

dQQ_rocket = OM*Q_rocket';

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
dY(10:13) = dQQ_rocket;
dY(14:16) = Vels_para;
dY(17) = du_para;
dY(18) = dv_para;
dY(19) = dw_para;
dY(20:22) = [p_rocket q_rocket r_rocket];
dY=dY';

%% Eigenvalues solution

% States equations
F = [dY];

% Vector of unknowns
X = [x_rocket y_rocket z_rocket u_rocket v_rocket w_rocket p_rocket q_rocket r_rocket...
    q0_rocket q1_rocket q2_rocket q3_rocket x_para y_para z_para u_para v_para z_para ...
    theta_x theta_y theta_z];

% Jacobian of the system
A = jacobian(F,X);

% Substitue reference conditions
A = subs(A, {x_rocket y_rocket z_rocket u_rocket v_rocket w_rocket p_rocket q_rocket r_rocket...
    q0_rocket q1_rocket q2_rocket q3_rocket x_para y_para z_para u_para v_para z_para ...
    theta_x theta_y theta_z}, X0');

% Conver into doubles
A = double(A);

% Eigenvalues evaluation
lambda = eig(A);

%% Post-Process

% Plot eigenvalues
figure('Name','Eigenvalues of the system','NumberTitle','off')
hold on, grid minor, box on
plot(lambda,'o','MarkerSize',10,'LineWidth',2)
plot([0 0],[-1.2*max(abs(imag(lambda))) 1.2*max(abs(imag(lambda)))],'k')
plot([-1.2*max(abs(real(lambda))) 1.2*max(abs(real(lambda)))],[0 0],'k')
ylim([-1.2*max(abs(imag(lambda))) 1.2*max(abs(imag(lambda)))])
xlim([-1.2*max(abs(real(lambda))) 1.2*max(abs(real(lambda)))])
% title('Eigenvalues of the system')
xlabel('Real(\lambda)','Fontsize',14,'interpreter', 'tex');
ylabel('Im(\lambda)','Fontsize',14,'interpreter', 'tex');
% xlabel('\Re_{\lambda}','interpreter', 'tex');
% ylabel('\Im_{\lambda}','interpreter', 'tex');

% Stability of the system
if real(lambda) < 0             
    fprintf('The system is asimptotically stable because all\n')
    fprintf('the eigenvalues are less than zero \n\n')
elseif real(lambda) <= 0
    fprintf('The system is simply stable because one or more eigenvalue has\n') 
    fprintf('real part equal to zero and the others are less than zero\n\n');
else
    fprintf('The system is unstable because one or more eigenvalue has\n') 
    fprintf('real part bigger than zero \n\n');
end

fprintf('Eigenvalues: \n');

% Eigenvalues print
fprintf('The eigenvalues of the linearized system are:\n ')
fprintf('L_1 -> %g \n L_2 -> %g \n L_3 -> %g \n L_4 -> %g \n ',lambda(1:4))
fprintf('L_5 -> %g \n L_6 -> %g \n L_7 -> %g \n L_8 -> %g \n ',lambda(5:8))
fprintf('L_9 -> %g \n L_10 -> %g \n L_11 -> %g \n L_12 -> %g \n ',lambda(9:12))
fprintf('L_13 -> %g \n L_14 -> %g \n L_15 -> %g \n L_16 -> %g \n ',lambda(13:16))
fprintf('L_17 -> %g \n L_18 -> %g \n L_19 -> %g \n L_20 -> %g \n ',lambda(17:20))
