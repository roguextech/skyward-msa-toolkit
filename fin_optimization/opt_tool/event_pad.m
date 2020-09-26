function [value, isterminal, direction] = event_pad(~, Y, settings, varargin)
% Event function to stop simulation at apogee

% Author: Ruben Di Battista
% Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
% email: ruben.dibattista@skywarder.eu
% Website: http://www.skywarder.eu
% April 2014; Last revision: 25.IV.2014
% License:  2-clause BSD

%Stop checking if I'm in Propulsion Phase

value = - Y(3) - settings.lrampa*sin(settings.OMEGA);

isterminal = 1;
direction = 1;


end

