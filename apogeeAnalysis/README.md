# apogeeAnalysis
This is a code developed in MATLAB for a primary apogee
analysis with different motors when the structural mass
is only known with a degree of uncertainty.
The simulator computes the different apogees, maximum
accelerations and launchpad exit velocities for
different motors and different masses.

# Usage
The parameters used in this code are stored in the
"settings" structure, and can be changed from there.
Before starting the simulation, some parameters must be
changed in the 'configApogee.m'. For instance,
a total impulse range must be chosen. It allows
the simulator to pick themotors that are in this range
from the motors matrix ('Motors.mat').
The deviation from the structural mass and the amount
of mass points in this deviation must also be chosen
in order to have a different range of masses.
The plots of maximum acceleration and launchpad exit
velocity can be chosen by setting
'settings.accelerationPlot = true' and
'settings.launchpadVelPlot = true'.
For both 'upwind' and 'downwind' cases wind magnitude,
wind elevation and wind azimuth must be chosen.
For both cases the aerobrakes height must be chosen
too (1= closed, 2= 50% open, 3= fully open).


# How it works and outputs
Once the simulation is started, it runs
'start_simulation.m', which gives apogees, maximum
accelerations and launchpad exit velocities for
every motor, mass, aerobrakes and wind condition as
outputs.
This code will then plot a number of different figure,
which will be a combination of the wind and aerobrakes
condition chosen.
In every figure there will be three subplots where
apogees, maximum accelerations and launchpad exit
velocities will be plotted for every motor in every
mass point. There will also be a legend, where the
motors will be listed in the same order (and color)
they appear in the apogee subplot.
