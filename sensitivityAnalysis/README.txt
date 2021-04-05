# sensitivityAnalysis

This code implements a sensitivity analysis on the ascent phase of the rocket.
Two types of analysis are available: derministic and stochastic.

In the deterministic analysis it is possible to vary the nominal values of the 
aerodynamics coefficients and the structural mass of the rocket. The relative 
magnitude of the variations is set by the user and it is the same for all the
parameters considered in the analysis. 
Several simulations are performed, in each one of these only one input parameter is
perturbed and its magnitude remains costant during the simulation.
The output of the deterministic analysis is a set of plots in which the value of 
several output parameters of the simulations (for example the apogee) is shown as a 
function of the relative variation of the input parameters.

In the stochastic analysis a Monte Carlo simulation is performed. Several 
simulations are performed, in each of these the uncertain parameters assume values
according to a normal distribution centered on their respective nominal values. 
The relative standard deviation of each uncertain parameter is set by the user.
The uncertain parameters are: the structural mass, the axial force coefficient, and 
the thrust. 
The output of the analysis is a plot of the distribution of the apogee altitude, 
and the mean value and the standard deviation of both the apogee altitude and the 
apogee time.

# Usage

The analysis type and its features can be set in the script configSensitivity.
The nominal values of all the parameters are stored in the settings structure.

- Deterministic Analysis:
To start a deterministic analysis set the variable settings.sensitivity.stoch as
false.
The varying parameters can be selected using the variable settings.sensitivity.para.
The latter is a cell which contains the list of the parameters' names. 
For example: set settings.sensitivity.para = {'CA', 'CNA', 'ms'} to vary the CA and 
CNA coefficients and the structural mass.
The values of relative variation to be applied to each parameter are stored in the 
variable settings.sensitivity.deltaDet.

- Stochastic Analysis:
To start a stochastic analysis set the variable settings.sensitivity.stoch as
true.
The number of simulations to be run is stored in the variable settings.sensitivity.N.
The relative standard deviation of each parameter is stored in the variable
settings.sensitivity.stdStoch.
The variable settings.sensitivity.thrustUncertainty can assume two values:
	- "same": to apply the same uncertainty to all the points of the experimental
			  thrust curve.
	- "independent": to apply the uncertainty considering and indipendent distribution 
					 for each experimental point.
