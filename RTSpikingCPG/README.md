# Real-Time Spiking CPG

# Summary

This code implements an SNS Toolbox object as a 4-neuron CPG that outputs spikes and membrane voltages of each neuron. The CPG is comprised of 2 non-spiking neuron half-centers that each output to a spiking neuron tuned to output at very low frequencies (up to ~50 Hz). 

## Dependencies

- SNS Toolbox, available at https://github.com/wnourse05/SNS-Toolbox/tree/master

- Readline.py, contained in this folder. Readline.py is a workaround for the sluggish behavior of the pySerial serial.readline() command.