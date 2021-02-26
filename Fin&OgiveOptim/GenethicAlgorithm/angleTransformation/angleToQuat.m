function q = angleToQuat(yaw, pitch, roll)

dcm = zeros(3);
cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);


dcm(1, 1) = cp.*cy;
dcm(1, 2) = cp.*sy;
dcm(1, 3) = -sp;
dcm(2, 1) = sr.*sp.*cy - cr.*sy;
dcm(2, 2) = sr.*sp.*sy + cr.*cy;
dcm(2, 3) = sr.*cp;
dcm(3, 1) = cr.*sp.*cy + sr.*sy;
dcm(3, 2) = cr.*sp.*sy - sr.*cy;
dcm(3, 3) = cr.*cp;

q = dcm2quat(dcm);