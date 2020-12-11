
close all; clc; clear;

%%
load('data.mat')
lat0 = 41.810093;        
lon0 = 14.052546;      
h0 = 1416;

X = [data_ascent.state.Y(:, 1); data_bal.state.Y(:, 1)];
Y = [data_ascent.state.Y(:, 2); data_bal.state.Y(:, 2)];
Z = [data_ascent.state.Y(:, 3); data_bal.state.Y(:, 3)];
T = [data_ascent.state.T; data_bal.state.T];
[T, iT, ~] = unique(T); X = X(iT); Y = Y(iT); Z = Z(iT);
N = 1000; Tinterp = linspace(T(1), T(end), N);
X = interp1(T, X, Tinterp);
Y = interp1(T, Y, Tinterp);
Z = interp1(T, Z, Tinterp);

[lat, lon, H] = ned2geodetic(X, Y, Z, lat0, lon0, h0, wgs84Ellipsoid);

%% mars-earth orbits

vName = 'mediaVideoTrajectory';
v = VideoWriter(vName, 'MPEG-4');
v.FrameRate = 60;
open(v);

uif = uifigure;
g = geoglobe(uif);
geoplot3(g, lat0, lon0, h0+100, 'ro', 'MarkerSize', 12);
hold(g,'on');
rocket = geoplot3(g, lat(1), lon(1), H(1), 'bo', 'MarkerSize', 7);
campos(g, lat0+0.01, lon0-0.06, 6000);
camheading(g, 100);
campitch(g, -35);

for i = 1:10:N
    rocket.LatitudeData = lat(i);
    rocket.LongitudeData = lon(i);
    rocket.HeightData = H(i);
    
%     h2.XData = RR2(i, 1);
%     h2.YData = RR2(i, 2);
%     h2.ZData = RR2(i, 3);
    
    drawnow
    
    frame = getframe(gcf);
    writeVideo(v, frame);
    
end
close(v);
close(vName)