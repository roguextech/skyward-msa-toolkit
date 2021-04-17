function funZ = funZGen(fn_map, lat0, lon0)
%{
 funZ - This function tells the altitude of landing point at Roccaraso
 INPUTS:
 -fn_map,    filename of the map 
 -lat0,      double [1, 1], latitude of the origin
 -lon0,      double [1, 1], longitude of the origin

 OUTPUTS
 -z,         double [1, 1], altitude of the landing point:
                             -if mode=='xy' --> z=funZ(x,y) in a NED system
                             -if mode=='LL' --> z=funZ(lat,lon) in a wgs84 based LLA system
 CALLED FUNCTIONS: /
%}
load(fn_map, 'RA', 'ZA', 'latlim', 'lonlim')
if lat0 < latlim(1) || lat0 > latlim(2) || lon0 < lonlim(1) || lon0 > lonlim(2)
    error('the initial coordinates are outside the range of the zData.map coordinates, check the config')
end
    

lat = RA.LatitudeLimits(2) - (RA.YIntrinsicLimits(1):(RA.YIntrinsicLimits(2)-1))*RA.CellExtentInLatitude;
lon = RA.LongitudeLimits(1) + (RA.XIntrinsicLimits(1):(RA.XIntrinsicLimits(2)-1))*RA.CellExtentInLongitude;
jmax = length(lon);
imax = length(lat);
LON = zeros(imax*jmax, 1);
LAT = zeros(imax*jmax, 1);
ZOO = zeros(imax*jmax, 1);

for j = 1:jmax
    LON((imax*(j - 1) + 1):(imax*j)) = ones(imax, 1)*lon(j);
    LAT((imax*(j - 1) + 1):(imax*j)) = lat;
    ZOO((imax*(j - 1) + 1):(imax*j)) = ZA(:,j);
end

h0 = relative*geointerp(ZA, RA, lat0, lon0);
[xNorth, yEast, zDown] = geodetic2ned(LAT, LON, ZOO, lat0, lon0, h0, wgs84Ellipsoid);
funZ = scatteredInterpolant(xNorth, yEast, zDown);

