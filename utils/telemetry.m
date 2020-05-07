close all,
clear all,
clc

% load('V1_sim.mat');
% t = time;
% N = length(t);
% axN = ax_b; % Add static acceleration (in BODY frame)
% [~, ~, Press,~]= atmosisa([data_ascent.interp.alt']);

%% READ HERMES V1 TELEMETRY
telemetryFileName = 'hr_tm_v1.csv';
rocketName = 'Hermes V1';
T1 = readtable(telemetryFileName);
startInd = 42288; % Definisci il campione a cui partono le misurazioni
endInd = size(T1,1);
% Estrai dati
time = T1.timestamp(startInd:end)./1000;
P_ana = T1.pressure_ada(startInd:endInd);
P_digi = T1.pressure_digi(startInd:endInd);
alt_agl = T1.agl_altitude(startInd:endInd);
alt_msl = T1.msl_altitude(startInd:endInd);
alt_gps = T1.gps_alt(startInd:endInd);
lon = T1.gps_lon(startInd:endInd);
lat = T1.gps_lat(startInd:endInd);
p = T1.gyro_z(startInd:endInd);
q = T1.gyro_y(startInd:endInd);
r = T1.gyro_x(startInd:endInd);
acc_x = T1.acc_z(startInd:endInd);
acc_y = T1.acc_y(startInd:endInd);
acc_z = T1.acc_x(startInd:endInd);
vxb = T1.vert_speed(startInd:endInd);

time = time - time(1);

% ANGLES DURING ASCENT
pitch_angle = zeros(length(time),1);
yaw_angle = zeros(length(time),1);
roll_angle = zeros(length(time),1);
pitch_angle(1) = 87; % Inclinazione rampa
for k = 2:length(time)
    pitch_angle(k) = pitch_angle(k-1) + (q(k) + q(k-1))/2*(time(k)-time(k-1));
    yaw_angle(k) = yaw_angle(k-1) + (r(k) + r(k-1))/2*(time(k)-time(k-1));
    roll_angle(k) = roll_angle(k-1) + (p(k) + p(k-1))/2*(time(k)-time(k-1));
end
pitch0 = pitch_angle(end);
yaw0 = yaw_angle(end);
roll0 = roll_angle(end);

% Find burning time and thrust curve
[aMaxD, index_MaxD] = min(acc_x); % Min acc -> Max Drag
tmax_drag = time(index_MaxD);
aMaxD = abs(aMaxD);
aD = aMaxD.*(time(1:index_MaxD)./tmax_drag).^2; % Assumi che l'accelerazione
                                                % dovuta alla drag sia
                                                % parabolica nel tempo
accT = acc_x(1:index_MaxD)+aD; % accelerazione dovuto alla thrust
ind_tburn = (find(accT>=0,1,'last')); % fine burning 
tburn = time(ind_tburn);

mprop = 0.9;
mfr = mprop/tburn;
m0 = 7.4;
mass = m0 - mfr*time; % Vettore massa
mass(ind_tburn+1:end) = m0 - mprop; % Massa a vuoto
exp_thrust = mass(1:ind_tburn).*accT; % Vettore spinta
Itot = trapz(time(1:ind_tburn),exp_thrust); % Impulso totale

% Calcola forze
% XForce = mass.*acc_x; %In generale
% YForce = mass.*acc_y; %In generale
% ZForce = mass.*acc_z; %In generale
% XAeroForce = XForce-

% Plot acceleration
figure, hold on, grid on
plot(time,acc_x);
plot(time,acc_y);
plot(time,acc_z);
xlabel('time [s]'), ylabel('acceleration [m/s^2]');
legend('x','y','z');
title(strcat('Body acceleration',{' of '},rocketName));

% Plot thrust
figure, plot(time(1:ind_tburn),exp_thrust),
title('Estimated thrust'); 
legend(rocketName);
ylabel('Thrust [N]');
xlabel('time [s]');
fprintf('Estimated tot. impulse of %s from telemetry: %.2f \n',rocketName,Itot);

% Vertical speed
figure; hold on, grid on
plot(time,vxb,'r');
xlabel('time [s]'), ylabel('v_x');
title('Vertical velocity');

% Angles
figure(); title('V1 Euler angles');
subplot(3,1,1), plot(time, pitch_angle); grid on
xlabel('time [s]'), ylabel('pitch angle [deg]');
title('Pitch angle');
subplot(3,1,2), plot(time, yaw_angle); grid on
xlabel('time [s]'), ylabel('yaw angle [deg]');
title('Yaw angle');
subplot(3,1,3), plot(time, roll_angle); grid on
xlabel('time [s]'), ylabel('roll angle [deg]');
title('Roll angle');

% Plot Altitude
figure; hold on, grid on
plot(time, alt_gps-alt_gps(1))
plot(time, alt_agl)
plot(time, alt_msl-alt_msl(1))
xlabel('time [s]'), ylabel('Altitude');
legend('gps','agl','msl');
title('Altitude');

[apo, apo_i] = max(alt_gps);

% Plot Trajectory
figure,
plot(lon,lat); hold on, grid on, axis equal
plot(lon(1),lat(1),'xr');
plot(lon(end),lat(end),'ok'); 
plot(lon(apo_i),lat(apo_i),'or')
plot(lon(ind_tburn),lat(ind_tburn),'dg');
legend('Trajectory','Launch pt','Landing pt','Apogee','Burning time');
xlabel('Longitude [deg]');
ylabel('Latitude [deg]');
title('Trajectory');

% Plot Pressure
figure, title('Pressure'); hold on
plot(time,P_digi./1e2);
plot(time,P_ana./1e2);
xlabel('time [s]');
ylabel('Pressure [mbar]');
legend('Digital','Analog');

%% READ HERMES V0 TELEMETRY
telemetryFileName = 'hr_tm_v0.csv';
rocketName = 'Hermes V0';
T0 = readtable(telemetryFileName);
startInd = 25534;
endInd = size(T0,1);
time2 = T0.timestamp(startInd:endInd)./1000;
acc_x_v0 = T0.z_acc(startInd:endInd);
time2 = time2 - time2(1);

% Find burning time and thrust curve
[aMaxD, index_MaxD] = min(acc_x_v0);
tmax_drag = time2(index_MaxD);
aMaxD = abs(aMaxD);
aD = aMaxD.*(time2(1:index_MaxD)./tmax_drag).^2;
accT = acc_x_v0(1:index_MaxD)+aD;
 
ind_tburn_v0 = (find(accT>0,1,'last'));
tburn = time2(ind_tburn_v0);
mprop = 0.9;
mfr = mprop/tburn;
m0 = 8.5;
mass = m0 - mfr*time2(1:ind_tburn_v0);
Tr_V0 = mass.*accT(1:ind_tburn_v0);
Itot = trapz(time2(1:ind_tburn_v0),Tr_V0);
figure, plot(time2(1:ind_tburn_v0),Tr_V0);
title('Estimated Thrust'); legend(rocketName);
ylabel('Thrust [N]');
legend(rocketName);
xlabel('time [s]');
fprintf('Estimated tot. impulse of %s from telemetry: %.2f \n',rocketName,Itot);


%% COMPARE PLOTS
% Compare X acceleration 
figure; hold on, grid on
plot(time,acc_x);
plot(time2,acc_x_v0);
xlabel('time [s]'), ylabel('ax');
legend('V1','V0');
title('X body acceleration')

