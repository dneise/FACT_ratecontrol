# FACT_ratecontrol

In this repository I tracked the work on an alternative version of FACT++ ratecontrol.cc. 
As it does not control the rates, but actually controls the thresholds 
(according to the measured sensor currents) in order to keep the rates "stable", 
it should maybe be named **thresholdcontrol.cc** now ...

The accidental trigger rate dominates the trigger rate of FACT only ~10% of the triggers are due to cosmic ray showers.
It is a function of the ambient photon rate (basically a poisson process) and the "trigger threshold",
which is proportional to the number of coincident photons within 
a time window of roughly 5 ns needed to trigger the telescope.

The well tested `ratecontrol.cc` keeps the accidental trigger rate stable even under varying ambient light 
conditions. By modeling the necessary patch threshold as a function of the measured patch current 
as found by `ratecontrol.cc` one can speed up the threshold control process by only measuring the current and setting the 
threshold accordingly. 

This modelling has been done by studying historic data over the course of one year in 
[a sperate study](https://github.com/dneise/current_vs_threshold/blob/master/the_real_thing/README.md).
The result of that study can be found 
[above the CalcThresholdsFromCurrents() function](https://github.com/dneise/FACT_ratecontrol/blob/master/threshold_from_currents.cpp#L342).

Each of the 320 bias patches forming the FACT image sensor is assigned a function to derive the necessary 
patch threshold from. Since two bias patches form a single trigger patch, one finds for the two bias currents also two corresponding trigger thresholds. Since the "brighter" patch will dominate the accidental trigger rate of that patch, also the threshold is taken from that patch:

    trigger_patch_threhold = max(bias_patch_with_4_pixel, bias_patch_with_5_pixel)

c.f. [CombineThresholds()](https://github.com/dneise/FACT_ratecontrol/blob/master/threshold_from_currents.cpp#L387)

For the 4 non-standard patches, the threshold is always taken from the neighboring standard-bias-patch, which is luckily always available, i.e. there is not "broken" bias patch whose neighboring bias-patch is also broken.
However, if there is a star inside a bias-patch containing a crazy-pixel, this is not reflected in the threshold for this patch, so there will be an effect visible for bright stars in so called "crazy-patches", 
c.f. [CombineThresholds() 2nd part.](https://github.com/dneise/FACT_ratecontrol/blob/master/threshold_from_currents.cpp#L397)

# Changes with respect to the original ratecontrol.cc

There are many, basically this is a new program. 
However in order to retain its orignal interface 
care has been taken to behave as similar as possible to the original `ratecontrol.cc`.

## No Global Threshold Setting

The original ratescan.cc would also use the currents to find the apropriate theshold
(just like this new alternate version does),
but it would use the median current to find a global threshold for all patches as a kind of "starting point". 
This step was called "Setting Global Threshold" and does not exist anymore in the new version.

## No 
