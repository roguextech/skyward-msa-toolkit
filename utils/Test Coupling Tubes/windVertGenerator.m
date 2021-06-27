function [ww] = windVertGenerator(MagMin, MagMax)
%{

windVertGenerator - function that generates vertical wind component

INPUTS:
- MagMin,      double [1, 1], Minimum wind magnitude, [m/s];
- MagMax,      double [1, 1], Maximum wind magnitude, [m/s].

OUTPUTS:
- ww,          double [1, 1], wind component along z.

CALLED FUNCTIONS: /

REVISION:
-#0, 17/01/2016, Release, Ruben Di Battista & Gabriele Poiana

%}

%Generating random value for magnitude

x = rand;
ww = MagMin + (MagMax - MagMin)*x;
if x < 0.5
    ww = -ww; 
end
end

