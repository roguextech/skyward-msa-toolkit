# autoMatricesProtub

This code allows an automatic computation of the rocket aerodynamic coefficients
using Missile DATCOM.

Fixed geometric variables are taken from SimulationsData.m located in ../data
folder. Those variables that can not be prior known, like flight conditions,
are taken from configAutoMatProtub.

The output data are stored in two MAT-file named "full" and "empty", representing
the two main configurations assumed by the rocket during flight: with propellant
and without.

Computational time relies on the amount of cases defined in the script 
configAutoMatProtub. However, it never takes more than 15 seconds to reach the aim.


# Usage 

Like previously reported, variables range can be set in the script
configAutoMatProtub, while the other quantities needed are stored in 
simulationData script.
