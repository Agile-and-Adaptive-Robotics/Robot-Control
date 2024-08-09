# Real-Time Spiking CPG

# Summary

This code implements a Python-Arduino serial data pipeline for implementing pulse-based control of AARL braided pneumatic actuator (BPA) muscles. The neural controller is an SNS Toolbox object as a 4-neuron CPG that outputs spikes and membrane voltages of each neuron. The CPG is comprised of 2 non-spiking neuron half-centers that each output to a spiking neuron tuned to output at very low frequencies (up to ~50 Hz). 

The Arduino code is Teensy-specific and actuates two BPAs, then returns the transmitted signal for latching purposes. 

## Dependencies

- SNS Toolbox, available at https://github.com/wnourse05/SNS-Toolbox/tree/master

- Readline.py, contained in this folder. Readline.py is a workaround for the sluggish behavior of the pySerial serial.readline() command.

- Py_Ard_Pipeline_Mult_Muscles.ino, contained in this folder. Py_Ard_Pipeline_Mult_Muscles.ino is a Teensy-specific Arduino code that acts as the other half of the data pipeline. The Teensy implements pulse-based control of two outputs. 
