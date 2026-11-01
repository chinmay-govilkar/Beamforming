The repository contains these files:
1)beamform_das.py-> contains the code for the beamforming pipeline in python for easier understanding
2) DasMemoryMatrix-> Verilog module for reading and writing to the memory with approriate control signals
3) DasSingleChannel-> Verilog module for working with the samples brough in with the memory.Things like interpolation if and when needed, currently in the basic version it acts as a buffer
4) DasCalculation-> For calculation of exact fetch address for each line. It is a simple Time of Flight*Sampling Frequency implementation. Time of flight is calculated by calculating the distance between transducer at a specific postion and corresponding pixel for which we are accumulating addresses currently.
5) beamformer-> For top level control of all modules mentioned
What remains is writing the module for deciding how do the pixel addresses to the DasCalculation module go and then actual image formation.
