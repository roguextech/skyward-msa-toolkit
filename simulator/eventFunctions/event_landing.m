function [value, isterminal, direction] = event_landing(~, Y, settings, varargin)
%{

EVENT_LANDING - Event function to stop simulation at landing checking when a value tends to zero;
               the value taken is to account is the vertical position, z = 0 --> landing

INPUTS:     
            - t, integration time;
            - Y, state vector, [ x y z | u v w | p q r | q0 q1 q2 q3 | m | Ixx Iyy Izz ]:

                                * (x y z), NED{north, east, down} horizontal frame; 
                                * (u v w), body frame velocities;
                                * (p q r), body frame angular rates;
                                * m , total mass;
                                * (Ixx Iyy Izz), Inertias;
                                * (q0 q1 q2 q3), attitude unit quaternion.

            - settings, rocket data structure.

OUTPUTS:        
            - isterminal, logical input to stop the integration;
            - direction, to select the sign that the function must have when tends to zero, 1 = positive;
            - value, selected value to check if the integration has to be stopped (vertical position).

Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu
April 2014; Last revision: 25.IV.2014

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Last Revision: 19/05/2018

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

