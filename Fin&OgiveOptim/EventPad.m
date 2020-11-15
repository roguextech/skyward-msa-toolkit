function [value, isterminal, direction] = EventPad(~, Y, settings, varargin)
%{ 
Event function to stop simulation at the launchpad exit

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Update date: 21/10/20
%}

value = - Y(3) - settings.lrampa*sin(settings.OMEGA);

isterminal = 1;
direction = 1;


end

