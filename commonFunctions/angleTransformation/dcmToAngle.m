function [yaw, pitch, roll] = dcmToAngle(dcm)
%{
dcmToAngle - This function converts direction cosine matrix (dcm) to Euler
             angles.

INPUTS:
        - dcm, double [3,3], direction cosine matrix.

OUTPUTS:
        - yaw, double [1,1], rotation around z-axis;
        - pitch, double [1,1], rotation around y-axis;
        - roll, double [1,1], rotation around x-axis.

CALLED FUNCTIONS: -

REVISIONS:
-
%}

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