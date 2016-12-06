# FACT_ratecontrol

In this repository I tracked the work on an alternative version of FACT++ ratecontrol.cc. 
As it does not control the rates, but actually controls the thresholds 
(according to the measured sensor currents) in order to keep the rates "stable", 
it should maybe be named **thresholdcontrol.cc** now ...

# Abscract

The accidental trigger rate dominates the trigger rate of FACT only ~10% of the triggers are due to cosmic ray showers.
It is a function of the ambient photon rate (basically a poisson process) and the "trigger threshold",
which is proportional to the number of coincident photons within 
a time window of roughly 5 ns needed to trigger the telescope.

The well tested `ratecontrol.cc` keeps the accidental trigger rate stable even under varying ambient light 
conditions. By modeling the necessary patch threshold as a function of the measured patch current 
as found by `ratecontrol.cc` one can speed up the threshold control process by only measuring the current and setting the 
threshold accordingly. 

This modelling has been done by studying historic data over the course of one year in 
[a sperate study](https://github.com/dneise/current_vs_threshold/blob/master/the_real_thing/README.md)
