clc; clear; close all
run('config_R2A_hermes.m')
load funZ.mat
settings.funZ = funZ;

heights = 200:100:800;
magn_w = 4:10;
Az_launch = 0:90:270;

Nh = length(heights);
Nw = length(magn_w);
Na = length(Az_launch);
R_op = zeros(Nh,Nw,Na);
R_land = zeros(Nh,Nw,Na);

for k = 1:Na
    for i = 1:Nh
        for j = 1:Nw
            settings.wind.MagMin = magn_w(j);
            settings.wind.MagMax = magn_w(j);
            settings.PHImin = Az_launch(k)*pi/180; %[rad] Minimum Azimuth Angle from North Direction, user input in degrees (ex. 90)
            settings.PHImax = Az_launch(k)*pi/180; 
            settings.wind.AzMin = (Az_launch(k) + 180)*pi/180;
            settings.wind.AzMax = (Az_launch(k) + 180)*pi/180;
            settings.zdrg2 = heights(i);
            [Tf,Yf,Ta,Ya,bound_value] = stdRun(settings);
            %         t_rog_op = bound_value.td2;
            %         index = find(Tf == t_rog_op);
            % Save rogallo point opening
            R_op(i,j,k) = norm([bound_value.Xd2(1) bound_value.Xd2(2)]);
            % Save rogallo point landing
            R_land(i,j,k) = norm([Yf(end,1),Yf(end,2)]);
        end
    end
end


% R_round = round(R_land,2,'significant');
% varNames = strcat('h_Rog_', string(200:100:1200));
% varNames = arrayfun(@(x)char(varNames(x)),1:numel(varNames),'uni',false);
% RowNames = strcat('w_ ', string(1:10));
% RowNames = arrayfun(@(x)char(RowNames(x)),1:numel(RowNames),'uni',false);
% T = array2table(R_round','RowNames',RowNames,'VariableNames',varNames);