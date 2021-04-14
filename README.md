# msa-toolkit
The  ***MSA  toolkit***  is  the  repository  in  which  the  codes  implemented  by the Mission Analysis team  are  stored. It is composed by several folders that will be briefly introduced in the following paragraphs.

## simulator
This is a code developed in MATLAB for the simulation of 6 d.o.f. rocket dynamics. The simulator predicts 3D trajectory, apogee, forces acting on the rockets, and various other aerodyanmics data. 

## data
Folder with the current flight data, rocket geometry and simulation parameters.

## commonFunctions
In this folder, the common functions employed in the codes of the toolkit are stored.

## autoMatricesProtub
This  code  allows  an  automatic  computation  of  the  rocket  aerodynamic  coefficientsusing Missile DATCOM, for different aerobrakes configuration.

## aerodynamicsOptimization
This code implements an aerodynamics optimization of the rocket.  The optimization variables are the fin chords and heigth,  the fin shape,  the ogive length and the ogive shape.  The code uses the genetic algorithm to reach the aim.

## apogeeAnalysis
This code implements a primary apogee analysis with different motors when the structural mass is known with a degree of uncertainty, in order to chose the best one.

## sensitivityAnalysis
This code implements a sensitivity analysis on the ascent phase of the rocket. Two types of analysis are available: deterministic and stochastic. <br/>
In the deterministic analysis it is possible to vary the nominal values of the aerodynamics coefficients and the structural mass of the rocket. 
The relative magnitude of the variations is set by the user and it is the same for all the parameters considered in the analysis. <br/>
In the stochastic analysis several simulations are performed, in each of these the uncertain parameters assume values according to a normal distribution centered on their respective nominal values.

## utils
Although the most important folders are in the main path, this folder contains work-alone tools, and some useful scripts.
