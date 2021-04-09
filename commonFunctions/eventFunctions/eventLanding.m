function [value, isterminal, direction] = eventLanding(~, Y, settings, varargin)
%{
eventLanding - Event function to stop simulation at landing checking when a
               value tends to zero; the value taken is to account is the 
               vertical position, z = 0 --> landing.

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

REVISION:
- #0    25/04/2014, Release, Ruben Di Battista

- #1    19/05/2018, Revision, Adriano Filippo Inno
%}

x = Y(1);
y = Y(2);
z = -Y(3);

if settings.terrain
    zloc = -settings.funZ(x,y);
    if zloc > 853
        zloc = 853;
    end
    
    if zloc < -656
        zloc = -656;
    end
    
    value = z - zloc;
else
    value = z;
end


isterminal = 1;
direction = 0;


end

