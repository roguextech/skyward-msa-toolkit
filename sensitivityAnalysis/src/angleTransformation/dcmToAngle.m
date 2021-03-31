function [yaw, pitch, roll] = dcmToAngle(dcm)

pitch = squeeze(-asin(dcm(1, 3, :)));
yaw = squeeze(atan2(dcm(1, 2, :), dcm(1, 1, :)));
roll = squeeze(atan2(dcm(2, 3, :), dcm(3, 3, :)));

singularities = abs(cos(pitch)) < 1e-2;

if any(singularities) 
    
    warning('Singularity condition for Euler angles is met. (cos(pitch) = 0)');
    
    pitch(singularities) = NaN;
    yaw(singularities) = NaN;
    roll(singularities) = NaN;
    
end