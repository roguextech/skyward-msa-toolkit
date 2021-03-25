function [yaw, pitch, roll] = dcmToAngle(dcm)

pitch = squeeze(-asin(dcm(1, 3, :)));
if cos(pitch) ~= 0
    yaw = squeeze(atan2(dcm(1, 2, :), dcm(1, 1, :)));
    roll = squeeze(atan2(dcm(2, 3, :), dcm(3, 3, :)));
else
    yaw = NaN;
    roll = NaN;
    warning('inverse transformation from dcm to Angle not defined for' +...
        'cos(pitch = 0)');
end