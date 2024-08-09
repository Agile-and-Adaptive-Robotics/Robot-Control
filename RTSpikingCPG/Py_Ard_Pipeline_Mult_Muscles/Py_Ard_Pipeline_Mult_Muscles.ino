#include <Arduino.h>
#include <Muscle.h>
/*
 * Py_Ard_Pipeline_Mult_Muscles
 *  
 *  This code implements a data pipeline in conjunction with RTSpikingCPG.py, 
 *   actuating two braided pneumatic actuator (BPA) muscles on robots in the
 *   Agile & Adaptive Robotics Laboratory (AARL) at Portland State University.
 *
 * Dependencies:
 *  - Muscle.h library, a library for implementing pulse-based control of AARL
 *      BPA muscles.
 * 
 *  - Arduino.h library, a library for encoding Teensy microcontrollers.
 *
 * Date: 8/9/2024
 * By: Stu McNeal
 */

// Define the control pins for the valves that control the BPA muscles. Specific to the AARL QTB. 
int pin_valve_hipEX = 3;//33
int pin_valve_hipFL = 4;//32

// Define variable for holding the hip joint angle
double angle_Hip = 0;

// Define the pressure sensor pins for each muscle object. Specific to the AARL QTB.
int pin_pressure_sensor_hipEX = 15;
int pin_pressure_sensor_hipFL = 21;

// Create the Muscle objects. The format for creating a Muscle class object is:
// Muscle <name>(String Name, int pin_valve, int pin_pressure_sensor)  
Muscle hipEX("hipEX", pin_valve_hipEX, pin_pressure_sensor_hipEX);
Muscle hipFL("hipFL", pin_valve_hipFL, pin_pressure_sensor_hipFL);

int x;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);

  //initialize Muscle object pins and set initial values (see Muscle.cpp) (Might be redundant...?)
  hipEX.begin();
  hipFL.begin();

  //Set maximum pulse frequency for the hipEX muscle
  hipEX.SetPulseFrequency(50);
  hipFL.SetPulseFrequency(50);
}

void  loop() {
  
  if (Serial.available()){

    // Read incoming integer value. Data is formatted as 9<hipEX_spike><hipFL_spike> 
    x = Serial.readString().toInt();

    // Extract spiking values. Values above zero start a pulse.
    unsigned int hipEX_spike = (x / 10U) % 10;
    unsigned int hipFL_spike = x % 10U;

    // If toggled, start hipEX pulse
    if(hipEX_spike){
      hipEX.ShouldPulseStart();
    } 

    // If toggled, start hipFL pulse
    if(hipFL_spike){
      hipFL.ShouldPulseStart();
    }

    // Return x for data pipeline
    Serial.println(x); 
  }

  // End pulses as needed
  hipEX.ShouldPulseEnd();
  hipFL.ShouldPulseEnd();

}
