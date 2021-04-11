function [value, isterminal, direction] = eventPad(~, Y, settings, varargin)
%{
eventPad - Event function to stop simulation at the launchpad exit

INPUTS:
- t,        double [1, 1], integration time  [s];
- Y,        double [4, 1], integration state, check launchPadFreeDyn for explanation; 
- settings, struct (motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind), 
                   simulation data;
- varargin, cell {3, 1}, for additional inputs of the ode function, un-necessary here.

OUTPUTS:
- value,        double [1, 1], see eventFunction explantion in matlab website
- isterminal,   double [1, 1], see eventFunction explantion in matlab website
- direction,    double [1, 1], see eventFunction explantion in matlab website

CALLED FUNCTIONS: /

REVISIONS:
- 0     21/10/20,   release     Adriano Filippo Inno
%}


value = - Y(3) - settings.lrampa*sin(settings.OMEGA);

isterminal = 1;
direction = 1;


end

