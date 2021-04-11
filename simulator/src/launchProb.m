function [p, flag, ind_Pin, ind_Pout, LPOP] = launchProb(settings, data_ascent, data_descent, LP)
%{
launchProb - This function allows to compute the probability of launch
subject to the constraints specified in config.m:

                - Stability Margin                                                     #1
                - Landing area                                                         #2
                - Crash of 1st drogue at opening (apogee + delay time of opening)      #3

INPUTS:         - settings, struct, rocket data structure;
                - data_ascent, cell, (n°simulations, 1), cell matrix containing fligth data of the ascent phase;
                - data_descent, cell, (n°simulations, n°parachutes), cell matrix containing fligth data of the descent phase;
                - LP, array, [n°simulations, 3], landing points.

OUTPUTS:
                - p, array, [1, 1], probability;
                - flag, array, [n° simulations, 1], outputs if each constrain is active as follows;
                - ind_Pin, logical array, [1, 1], logical vector that specifies il the i-th landing point is inside the landing area;
                - ind_Pout, logical array, [1, 1], logical vector that specifies il the i-th landing point is outside the landing area;
                - LPOP, array, [n° simulations, 2], last parachute opening points.

FLAGS (logical explanation):     #1  #2  #3
                                 1   1   1    ==> flag = 1
                                 1   1   0    ==> flag = -1
                                 1   0   1    ==> flag = -2
                                 0   1   1    ==> flag = -3
                                 1   0   0    ==> flag = -4
                                 0   1   0    ==> flag = -5
                                 0   0   1    ==> flag = -6
                                 0   0   0    ==> flag = -7

REVISIONS:
- #0 16/11/2018, Release, Adriano Filippo Inno
%}

N_simul = settings.stoch.N;
V_lim = settings.stoch.prob.VLim;
XCP_lim = settings.stoch.prob.XCPLim;

% pre-allocation
flag = zeros(N_simul,1);
dist = zeros(N_simul, 1);
LPOP = zeros(N_simul, 2);

% ellipse definition
a = settings.prob.SafeEllipse.a;
b = settings.prob.SafeEllipse.b;
x0 = settings.prob.SafeEllipse.x0;
y0 = settings.prob.SafeEllipse.y0;
alpha = settings.prob.SafeEllipse.alpha;

c = sqrt(b^2 - a^2);                        % [m] safety ellipse focii distance
F1 = [x0, y0+c];                            % [m] first safety ellipse not-rotated foci coordinates
F2 = [x0, y0-c];                            % [m] second safety ellipse not-rotated foci coordinates

R = [cosd(alpha), - sind(alpha)             % Rotation matrix
    sind(alpha),   cosd(alpha)];

F1_hat = R*F1';                             % [m] first rotated focii
F2_hat = R*F2';                             % [m] second rotated focii


for i = 1:N_simul
    
    flag_XCP = all(-data_ascent{i}.coeff.XCPlon(not(isnan(data_ascent{i}.coeff.XCPlon))) > XCP_lim);
    
    P = LP(:,[2,1]);
    dist(i) = norm(P(i,:) - F1_hat') + norm(P(i,:) - F2_hat');
    flag_land = dist(i) < 2*b;
    ind_Pin = find(dist < 2*b);
    ind_Pout = find(dist > 2*b);
    LPOP(i,:) = data_descent{i, 1}.state(1).Y(end, 1:2);
    
end

V_apo = norm(data_ascent{i}.state.Y(end, 4:6) - data_ascent{i}.wind.body_wind(1:3, end)');

flag_drogue1 = V_apo < V_lim;

if flag_XCP == 1
    
    if flag_land == 1
        
        if flag_drogue1 == 1
            flag(i) = 1;
        else
            flag(i) = -1;
        end
        
    else
        
        if flag_drogue1 == 1
            flag(i) = -2;
        else
            flag(i) = -4;
        end
        
    end
    
else
    
    if flag_land == 1
        
        if flag_drogue1 == 1
            flag(i) = -3;
        else
            flag(i) = -5;
        end
        
    else
        
        if flag_drogue1 == 1
            flag(i) = -6;
        else
            flag(i) = -7;
        end
        
    end
    
end

k = length(flag(flag == 1));
p = k/N_simul*100;
