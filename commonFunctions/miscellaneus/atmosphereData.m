function [T, a, P, rho] = atmosphereData(absoluteAltitude, g)


atmCostants = [0.0065, 401.87434, 1.86584515, 1.225, 101325, 288.15]; % atmosisa constants:

T = atmCostants(6) - atmCostants(1)*absoluteAltitude;
a = sqrt(T*atmCostants(2));
theta = T/atmCostants(6);

P = atmCostants(5)*theta.^(g/atmCostants(3));
rho = atmCostants(4)*theta.^((g/atmCostants(3)) - 1.0);


end

