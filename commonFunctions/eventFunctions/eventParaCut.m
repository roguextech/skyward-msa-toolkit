function [value, isterminal, direction] = eventParaCut(~, Y, settings)
%{

eventParaCut - Event function to stop simulation at the chosen altitude to cut the 
                 parachute, checking when a value tends to zero;
                 the value taken is to account is the vertical position respect to the chosen altitude,
                 z - zcut = 0 --> parachute cut

INPUTS:     
            - t, array, [n° variations, 1], integration time;
            - Y, array, [n° variations, 16], state vector;
                    State vector: [ x y z | u v w | p q r | q0 q1 q2 q3 | Ixx Iyy Izz ]:

                                * (x y z), NED{north, east, down} horizontal frame; 
                                * (u v w), body frame velocities;
                                * (p q r), body frame angular rates;
                                * (q0 q1 q2 q3), attitude unit quaternion;
                                * (Ixx Iyy Izz), Inertias;

            - settings, struct, stores data of the rocket and of the simulation.

OUTPUTS:        
            - isterminal, logical array [1, 1], logical input to stop the integration;
            - direction, logical array [1, 1], to select the sign that the function must have when tends to zero, 1 = positive;
            - value, logical array [1, 1], selected value to check if the integration has to be stopped (vertical position).

REVISIONS:
- #0 25/04/2014, Release, Ruben Di Battista
- #1 09/10/2019, Revision, Adriano Filippo Inno
%}

x = Y(1);
y = Y(2);
z = -Y(3);

if settings.stoch.N == 1
    para = settings.paraNumber;
else
    para = settings.stoch.para;
end

if settings.terrain
    zloc = -settings.funZ(x,y);
    if zloc > 859
        zloc = 859;
    end
    
    if zloc < -845
        zloc = -845;
    end
    
    value = z - zloc - settings.para(para).z_cut;
else
    value = z - settings.para(para).z_cut;
end

isterminal = 1;
direction = 0;

end

