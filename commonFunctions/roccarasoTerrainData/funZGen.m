function funZ=funZGen(fn_map,lat0,lon0,relative,mode)
%{
 funZ - This function tells the altitude of landing point at Roccaraso
 INPUTS:
 -fn_map,    filename of the map 
 -lat0,      double [1, 1], latitude of the origin
 -lon0,      double [1, 1], longitude of the origin
 -relative,  logicalArray, if true, the z is the altitude relative to the origin, otherwise is the wgs84 altitude
 -mode       logicalArray :
                            -if mode=='xy' output is in xy coordinates, 
                            -if mode=='LL' output in latitude and longitude
 OUTPUTS
 -z,         double [1, 1], altitude of the landing point:
                             -if mode=='xy' --> z=funZ(x,y) in a NED system
                             -if mode=='LL' --> z=funZ(lat,lon) in a wgs84 based LLA system
 CALLED FUNCTIONS: /
%}
load(fn_map,'RA','ZA')
lat=RA.LatitudeLimits(2)-(RA.YIntrinsicLimits(1):(RA.YIntrinsicLimits(2)-1))*RA.CellExtentInLatitude;
lon=RA.LongitudeLimits(1)+(RA.XIntrinsicLimits(1):(RA.XIntrinsicLimits(2)-1))*RA.CellExtentInLongitude;
jmax=length(lon);
imax=length(lat);
LON=zeros(imax*jmax,1);
LAT=zeros(imax*jmax,1);
ZOO=zeros(imax*jmax,1);
for j=1:jmax
    LON((imax*(j-1)+1):(imax*j))=ones(imax,1)*lon(j);
    LAT((imax*(j-1)+1):(imax*j))=lat;
    ZOO((imax*(j-1)+1):(imax*j))=ZA(:,j);
end
if strcmp(mode,'xy')||strcmp(mode,'XY')
    h0=0+relative*geointerp(ZA,RA,lat0,lon0);
    [xNorth,yEast,zDown] = geodetic2ned(LAT,LON,ZOO,lat0,lon0,h0,wgs84Ellipsoid);
    funZ = scatteredInterpolant(xNorth,yEast,zDown);
elseif strcmp(mode,'ll')||strcmp(mode,'LL')
    funZ = scatteredInterpolant(LAT,LON,-ZOO+relative*geointerp(ZA,RA,lat0,lon0));
end
end