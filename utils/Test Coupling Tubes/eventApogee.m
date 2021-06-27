function [value, isterminal, direction] = eventApogee(t, Y, settings, varargin)
%{
eventApogee - Event function to stop simulation at apogee checking when a 
              value tends to zero; the value taken is to account is the 
              vertical velocity, vy = 0 --> apogee

INPUTS:     
        - t, double [n° variations, 1], integration time, [s];
        - Y, double [n° variations, 16], state matrix,
                            [ x y z | u v w | p q r | q0 q1 q2 q3 | m | Ixx Iyy Izz ]:
                            * (x y z), NED{north, east, down} horizontal frame;
                            * (u v w), body frame velocities;
                            * (p q r), body frame angular rates;
                            * m , total mass;
                            * (Ixx Iyy Izz), Inertias;
                            * (q0 q1 q2 q3), attitude unit quaternion.
        - settings, struct, rocket and simulation data.

OUTPUTS:        
        - value, selected value to check if the integration has to be stopped (vertical velocity);
        - isterminal, logical input to stop the integration;
        - direction, to select the sign that the function must have when tends to zero, 1 = positive.

CALLED FUNCTIONS: -

REVISIONS:
- #0    25/04/2014, Release, Ruben Di Battista
%}

Q = Y(10:13)';

% Inertial Frame velocities
vels = quatrotate(quatconj(Q),Y(4:6)');

% Stop checking if I'm in Propulsion Phase
if t > settings.tb
    value = vels(3);
else
    value = 1;
end

isterminal = 1;
direction = 1;


end

