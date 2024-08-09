"""
RTSpikingCPG (Real-Time Spiking CPG) is a python class that combines a 
non-spiking CPG with spiking output neurons for the purpose of driving braided 
pneumatic actuator (BPA) muscles in the Agile & Adaptive Robotics Laboratory 
(AARL) at Portland State University. This is test code for troubleshooting 
timing and robotic actuation. 

The code creates a pipeline for controlling a pair of antagonistic muscles:
    
    Python --spikes--> Arduino --> Actuate 2 BPAs
      ^                   |
      |                   v
        <--confirmation--
    
NOTES: 
    
    1. The code utilizes the SNS Toolbox and an associated Tutorial, accessible at 

    https://github.com/wnourse05/SNS-Toolbox/blob/master/tutorials/Tutorial%208%20-%20Voltage-gated%20Ion%20Channels.ipynb
    
    2. Instructions on how to install the SNS Toolbox can be found at
    
    https://github.com/wnourse05/SNS-Toolbox/tree/master
    
    3. This code relies on Readline.py, a patch that overcomes pipeline slowing 
    that arises from use of serial.readline(). Readline.py should be included 
    in the folder containing this code.

date: 8/9/24
by: Stu McNeal
"""

from sns_toolbox.neurons import NonSpikingNeuronWithPersistentSodiumChannel, SpikingNeuron
from sns_toolbox.connections import NonSpikingSynapse
from sns_toolbox.networks import Network
from sns_toolbox.renderer import render
import numpy as np
import time
import serial
from ReadLine import ReadLine
import matplotlib.pyplot as plt
# import simpy.rt
from sns_toolbox.plot_utilities import spike_raster_plot 
# from scipy.io import savemat

from typing import Dict, Any 

class RTSpikingCPG:
    def __init__(self,
                    Cm  = 5,        # Membrane capacitance [nF]
                    Gm  = 1,        # Membrane conductance [uS]
                    Ena = 50,       # Na resting potential
                    Er  = -60,      # Neuron resting potential [mV]
                    R   = 20,       # Neuron membrane range [mV]
                    S   = 0.05,     # Na ion channel activation slope in mV^-1
                    Km  = 1,        # scaling value representing A_m in Szczecinski et al., 2017
                    Kh  = 0.5,      # scaling value representing A_h in Szczecinski et al., 2017
                    Em:int  = -40,      # Na activation channel resting potential
                    Eh:int  = -60,      # Na deactivation channel resting potential
                    tauHmax:int = 300,  # time constant
                    port:str = 'COM11',  
                    comms:bool = True): # toggles serial comms with Arduino
        
        #assign vars to class parameters
        self.params: Dict[str, Any] = {}
        self.params['Cm'] = Cm
        self.params['Gm'] = Gm
        self.params['Ena'] = Ena
        self.params['Er'] = Er
        self.params['R'] = R
        self.params['S'] = S
        self.params['Km'] = Km
        self.params['Kh'] = Kh
        self.params['Em'] = Em
        self.params['Eh'] = Eh
        self.params['tauHMax'] = tauHmax
        self.params['comms'] = comms

        #if toggled, initialize the serial channel
        if comms:
            self.arduino = serial.Serial(port=port, baudrate=115200, timeout=0.1)
            self.rl = ReadLine(self.arduino)

    def zinf(self, U, Kz, Sz, Ez):
        """Helper function for calculating embedded CPG functions"""
        return 1/(1+Kz*np.exp(Sz*(Ez-U)))

    def tauz(self, U, tauzmax, Kz, Sz, Ez):
        """Helper function for calculating embedded CPG functions"""
        return tauzmax*self.zinf(U, Kz, Sz, Ez)*np.sqrt(Kz*np.exp(Sz*(Ez-U)))
    
    def minf(self, U, Km, S, delEm):
        """Helper function for calculating embedded CPG functions"""
        return self.zinf(U, Km, S, delEm)

    def hinf(self, U, Kh, S, delEh): #S is -S here
        """Helper function for calculating embedded CPG functions"""
        return self.zinf(U, Kh, S, delEh) #S is -S here 
    
    def cpg(self, delta:float = 0.01, show:bool = False):
        """
        Returns net, an SNS Toolbox object containing a central pattern 
        generator comprised of two non-spiking cross-inhibiting half-center 
        neurons (H0, H1) that output to two spiking neurons (M0, M1). 
        
        Network inputs:
        ===============    
            inputs[0], initial cpg perturbation 
            
        Network outputs:
        ================
            data[0], H0 membrane voltage in mV 
            data[1], H1 membrane voltage in mV
            data[2], M0 spikes 
            data[3], M1 spikes
            data[4], M0 membrane voltage in mV
            data[5], M1 membrane voltage in mV
            
        Parameters:
        ===========
            float:
                delta, controls cpg oscillation speed. Default is 0.1.
            bool:
                show, toggles a rendering of the network. Default is False.
                
        Returns:
        ========
            class object:
                net, an SNS Toolbox neural network. 
        
        """
        #assign needed parameter values
        Cm      = self.params['Cm']
        Gm      = self.params['Gm']
        Ena     = self.params['Ena']
        Er      = self.params['Er']
        delEna  = Ena - Er
        R       = self.params['R']
        S       = self.params['S']
        Km      = self.params['Km']
        Kh      = self.params['Kh']
        Em      = self.params['Em']
        Eh      = self.params['Eh']
        delEm   = Em - Er
        delEh   = Eh - Er
        tauHmax = self.params['tauHMax']

        # Calculate sodium channel conductance
        Gna = Gm*R/(self.zinf(R, Km, S, delEm)*self.zinf(R, Kh, -S, delEh)*(delEna - R))

        # Set derived ion channel parameters
        g_ion   = [Gna]
        e_ion   = [delEna]

        pow_m   = [1]
        k_m     = [Km]
        slope_m = [S]
        e_m     = [delEm]

        pow_h   = [1]
        k_h     = [Kh]
        slope_h = [-S]
        e_h     = [delEh]
        tau_max_h = [tauHmax]

        
        
        # Calculate maximum contuctance of the half-center synapses 
        Ein = -100; delEsyn = Ein-R
        
        gSyn = (-delta - delta*Gna*self.minf(delta, Km, S, delEm)*self.hinf(delta, Kh, -S, delEh) + Gna*self.minf(delta, Km, S, delEm)*self.hinf(delta, Kh, -S, delEh)*delEna)/(delta - delEsyn)
        
        """ Create synapse templates """
        # Non-spiking synapse template
        synapse_cpg = NonSpikingSynapse(max_conductance         = gSyn, 
                                        reversal_potential      = delEsyn)

        # Non-spiking half center-to-spiking synapse template
        base_syn    = NonSpikingSynapse(max_conductance         = 0.078,
                                        reversal_potential      = 40,
                                        e_lo                    = 5,
                                        e_hi                    = 35)
        
        """ Create neuron templates """
        
        # Half-center neuron template
        neuron_cpg = NonSpikingNeuronWithPersistentSodiumChannel(
                            membrane_capacitance                = Cm, 
                            membrane_conductance                = Gm,
                            g_ion                               = g_ion,
                            e_ion                               = e_ion,
                            k_m                                 = k_m,
                            slope_m                             = slope_m,
                            e_m                                 = e_m,
                            k_h                                 = k_h,
                            slope_h                             = slope_h,
                            e_h                                 = e_h,
                            tau_max_h                           = tau_max_h,
                            name                                = 'HC',
                            color                               = 'orange')
        
        # Spiking neuron template
        spiking_neuron = SpikingNeuron(
                            threshold_time_constant             = 50  , # tau_m
                            threshold_initial_value             = 1.25, # theta_0
                            threshold_proportionality_constant  = 0   , # m
                            threshold_leak_rate                 = 1   ,
                            membrane_capacitance                = 32  ,
                            membrane_conductance                = 1   )

        """ Create the network"""
        net = Network()
        
        # Add neurons to the network
        net.add_neuron(neuron_cpg,      name='H0', color='blue')
        net.add_neuron(neuron_cpg,      name='H1', color='blue')
        net.add_neuron(spiking_neuron,  name='M0', color='orange')
        net.add_neuron(spiking_neuron,  name='M1', color='orange')
        
        # Add synapses to the network
        net.add_connection(synapse_cpg, 'H0', 'H1')
        net.add_connection(synapse_cpg, 'H1', 'H0')
        net.add_connection(base_syn,    'H0', 'M0')
        net.add_connection(base_syn,    'H1', 'M1')

        # Add input to H0 for initial perturbation
        net.add_input('H0')

        # Add outputs to the network 
        net.add_output('H0')
        net.add_output('H1')
        net.add_output('M0', spiking=True)
        net.add_output('M1', spiking=True)
        net.add_output('M0', spiking=False)
        net.add_output('M1', spiking=False)

        # If requested, display the network
        if show ==True:
            render(net)
            
        return net

    def run_sim(self, net, dt:float = 0.1, tMax:int = 500):

        #set simulation transient parameters
        I = 0
        tStart = 100
        tEnd = 400
        tTail = 250

        t = np.arange(0,tMax,dt)
        numSteps = np.size(t)

        Iapp = np.zeros(numSteps)
        # Iapp[tStart:tEnd] = I 

        Ipert = np.zeros(numSteps)
        Ipert[1] = 1

        #set up the compiler
        model = net.compile(backend='numpy', dt=dt)

        #generate data var to store model outputs
        data = np.zeros([len(t), net.get_num_outputs_actual()])
        inputs = Iapp + Ipert

        # Instantiate variables for real-time syncing. Instantiating 
        #  time_start variable (t_s) sets the reference time for the 
        #  time.perf_counter() clock
        t_s = time.perf_counter_ns()
        t_n = 0

        #run the simulation for the time specified
        for i in range(len(t)):

            # record the time, in nanoseconds (perf_counter reference point is 
            #  defined when t_s is first called, above)
            t_n = time.perf_counter_ns()

            # step the SNS model forward, populate the data block
            data[i] = model([inputs[i]])

            self.tstart = t_n/1e9 #converts to seconds

            #if selected, send data to the Arduino
            if self.params['comms']:
                
                #only send data if there is a spike...
                if int(data[i][2])==1 or int(data[i][3])==1:

                    #ensure that enough time has passed
                    while True:

                        #if enough time has passed since the last spike...
                        if t_n-t_s>= dt*i*1e6:

                            #...proceed
                            break

                        #otherwise...
                        else:
                            
                            #...update the time_now variable
                            t_n = time.perf_counter_ns()

                    #compile spikes into message
                    msg = 900 + int(''.join(map(str, [int(data[i][2]), int(data[i][3])])))

                    #send message to Teensy
                    self.arduino.write(bytes(str(msg), 'utf-8'))
                
                    #latch and read return message
                    try:
                        #read data from Arduino 
                        val = self.rl.readline()

                        #print vals (dev code)
                        print(str(val))
                        print(str(time.time()-self.tstart))

                    except:
                        raise Warning("Confirmation from Arduino not received")

        # reorient the data for plotting
        data = data.transpose()

        #if comms was toggled, close the channel
        if self.params['comms']:
            self.arduino.close()

        return t,data

    def py_arduino(self):
        pass

if __name__ == '__main__':
    
    # Generate the network object and call it "cpg". We set comms based on whether 
    #  we want to talk to the Arduino or just simulate the spiking oscillator
    cpg = RTSpikingCPG(comms=True, port='COM11')
    
    # Create the network
    net = cpg.cpg(delta=0.1, show=False)
     
    """ Set simulation increment, in ms. Cannot be less than minimum neuron 
    capacitance in Network. Minimum value on my computer: 0.2 ms. This value 
    should be set to be as small as possible while still allowing time-to-
    complete to be within a millisecond from the simulated time. These values 
    are printed at the end of every run. See below. """
    dt = 0.25 #[ms] 
    
    # Set the simulation length, in ms
    tMax = 10000
    
    # Start simulation timer (for verifying simulation time/real time alignmemt)
    tstart = time.time()

    # Run the simulation 
    t, data = cpg.run_sim(net, dt=dt, tMax=tMax)
    
    # Report elapsed times to the user
    print('Time to complete: '  + str(time.time()-tstart)   + ' s')
    print('Time simulated: '    + str(int(tMax/1000))       + ' s')

    # Plot half-center voltages
    plt.subplot(7,1,1)
    plt.plot(t,data[0,:],label='HC0',color='C0')
    plt.plot(t,data[1,:],label='HC1',color='C1',linestyle='--')
    plt.xlabel('t (ms)')
    plt.ylabel('U (mV)')
    plt.title('CPG')
    plt.legend()
    plt.xlim(0,tMax)
    
    # Plot M0 neuron spikes
    plt.subplot(7, 1, 3)
    plt.title('M0 spikes')
    spike_raster_plot(t, data[:][2],colors=['blue'])
    plt.xlim(0,tMax)
    
    # Plot M1 neuron spikes
    plt.subplot(7, 1, 5)
    plt.title('M1 spikes')
    spike_raster_plot(t, data[:][3],colors=['orange'])
    plt.xlim(0,tMax)
    
    # Plot spiking neuron voltages
    plt.subplot(7,1,7)
    plt.plot(t,data[4,:],label='MC0',color='C0')
    plt.plot(t,data[5,:],label='MC1',color='C1',linestyle='--')
    plt.xlabel('t (ms)')
    plt.ylabel('U (mV)')
    plt.xlim(0,tMax)

    # Save figure
    plt.savefig('Oscillating_spike_activations.png', bbox_inches='tight')
    
    # Show the figure
    plt.show()
    
    # Isolate the spike rates
    # x = np.zeros(len(data[:][2]))
    # y = np.zeros(len(data[:][3]))
    # rate_m0 = 1/(np.diff(data[:][2])*0.001)
    # rate_m1 = 1/(np.diff(data[:][3])*0.001)
    # rate_t = t[:-1]
    
    # Compile data into single variable for storage
    # dat = {"m0":data[:][2], "m1": data[:][3], "rate_m0": rate_m0, "rate_m1": rate_m1, "rate_t": rate_t}
    
    # Save data variable
    # savemat("spike_data.mat", dat)


