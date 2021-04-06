function [value, isterminal, direction] = eventMainExit(~, Y, settings)
%{

eventParaCut - Event function to stop simulation when the main parachute is extracted,
               checking when a value tends to zero;
               The value taken into account is the difference between the main parachute 
               relative position and the main chord's length (when it tends to zero the main parachute
               has completed the extraction)

    INPUTS:     
            - t, array, [n° variations, 1], integration time;
            - Y, array, [n° variations, 28], state vector;
                    State vector: [ x y z | u v w | p q r | q0 q1 q2 q3 | Ixx Iyy Izz | x_para1 y_para1 z_para1 | u_para1 v_para1 w_para1 | x_para2 y_para2 z_para2 | u_para2 v_para2 w_para2]:

                            * (x y z), NED{north, east, down} horizontal frame;
                            * (u v w), body frame velocities;
                            * (p q r), body frame angular rates;
                            * (q0 q1 q2 q3), attitude unit quaternion;
                            * (Ixx Iyy Izz), Inertias;
                            * (x_para1 y_para1 z_para1), NED{north, east, down} horizontal frame of the drogue;
                            * (u_para1 v_para1 w_para1), body frame velocities of the drogue;
                            * (x_para2 y_para2 z_para2), NED{north, east, down} horizontal frame of the main;
                            * (u_para2 v_para2 w_para2), body frame velocities of the main;

            - settings, struct, rocket data structure.

    OUTPUTS:        
            - isterminal, logical array [1,1], logical input to stop the integration;
            - direction, logical array [1,1], to select the sign that the function must have when tends to zero, 1 = positive;
            - value, logical array [1,1], selected value to check if the integration has to be stopped (difference between relative position of the main and its chord length).

    REVISIONS:
        - #0 16/12/2020, Release, Davide Rosato
%}
    para = settings.paraN;
    
    pos_para = [Y(23) Y(24) Y(25)];
    rel_pos = norm(pos_para - ([Y(1) Y(2) Y(3)] + ...
        quatrotate(quatconj([Y(10) Y(11) Y(12) Y(13)]),...
        [(settings.xcg(2)-settings.Lnose) 0 0])));
    
    value = settings.para(para(2)).L - rel_pos;
    
    isterminal = 1;
    direction = 0;
end