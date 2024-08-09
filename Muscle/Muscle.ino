#include <Arduino.h>
#include "Muscle.h"
/*
  Muscle_Blink.ino
  
  This sketch demonstrates the functionality of the Muscle library written 
  to control the muscles and attached sensors on the Quadruped and Quadruped 
  Test Bench robots in the Agile & Adaptive Robotics Laboratory (AARL) at 
  Portland State University. The goal of the library is to streamline the 
  interface. 
  
  Muscle_Blink demonstrates how to create a Muscle object and three methods of 
  controlling a pulsing algorithm: 
   
  Demo #1: [worst] Controls a single valve using delay(), mimicking the Blink() 
            function. Holds the cursor.   
  Demo #2: [better] Controls a single valve using millis(). A correct approach, 
            but verbose and difficult to scale. 
  Demo #3: [best] Controls any number of valves using the millis() approach from 
            Demo #2, but relies on versions of the code embedded in each Muscle 
            class object. The utility of this method is shown by its ability to 
            scale seamlessly to any number of Muscle class objects and pulse 
            profiles. Note that <muscle>.ShouldPulseStart() and 
            <muscle>.ShouldPulseEnd() can be consolidated into the single command 
            <muscle>.Pulse_Nanny(), which is available but not shown in this 
            example.
  
 Created 11 June 2024  
 By Stu McNeal
 */
 
// Define the pins for the valves that control the BPA muscles. QTB-specific. 
int pin_valve_hipEX = 33;
int pin_valve_hipFL = 34;

// Define the pressure sensor pins for each Muscle object. QTB-specific.
int pin_pressure_sensor_hipEX = 15;
int pin_pressure_sensor_hipFL = 21;

// Create the Muscle objects. The format for creating a Muscle class object is:
// Muscle <name>(String Name, int pin_valve, int pin_pressure_sensor)  
Muscle hipEX("hipEX", pin_valve_hipEX, pin_pressure_sensor_hipEX);
Muscle hipFL("hipFL", pin_valve_hipFL, pin_pressure_sensor_hipFL);

void setup() {  

  // Muscle.begin() sets the pinModes and instantiates the Muscle class state variables.
  hipEX.begin();
  hipFL.begin();
  
  // ================================================================================================
  // DEMO #1: Manual pulsing demo using delay(). Pulses hipEX muscle at 30 Hz for 30 seconds. 
  // ================================================================================================

  // define time parameters to control muscle pulses
  float dt_on = 9;                            //[ms]
  float freq = 45;                            //[Hz]
  float dt_off = 1000/freq - dt_on;           //[ms]
  unsigned long Now = millis();               //[ms]

  // pulse hipEX muscle at 30 Hz for 30 seconds
  while (millis() - Now < 30000){ 
    //open the valve
    hipEX.Open();

    //hold the cursor for dt_on ms
    delay(dt_on);

    //close the valve
    hipEX.Close();

    //hold the cursor for dt_off ms
    delay(dt_off);    
  }

  // ================================================================================================
  // Demo #2: Manual pulsing demo using millis(). Pulses hipFL muscle at 45 Hz for 30 seconds.
  // ================================================================================================
  
  // define time parameters to control muscle pulses
  dt_on = 9;                            //[ms]
  freq = 45;                            //[Hz]
  dt_off = 1000/freq - dt_on;           //[ms]
  Now = millis();                       //[ms]

  // pulse hipFL muscle at 45 Hz for 30 seconds
  while (millis() - Now < 30000){ 

    //decide whether to turn on the valve based on current valve position and whether dt_off time has elapsed
    if(!hipFL.IS_A_PULSE_ACTIVE && millis() - hipFL.WHEN_LAST_PULSE_ENDED > dt_off){

      //open the valve
      hipFL.Open();

      //record when the valve was opened
      hipFL.WHEN_THIS_PULSE_STARTED = millis();

      //flip the valve position boolean
      hipFL.IS_A_PULSE_ACTIVE = true;
    }
     //decide whether to close the valve based on current position and whether dt_on time has elapsed
     if(hipFL.IS_A_PULSE_ACTIVE && millis()-hipFL.WHEN_THIS_PULSE_STARTED > dt_on){

      //close the valve
      hipFL.Close();

      //record the time when the valve was closed
      hipFL.WHEN_LAST_PULSE_ENDED = millis();

      //flip the valve position boolean
      hipFL.IS_A_PULSE_ACTIVE = false;
     }    
  }
  //ensure that the valve finishes in the closed position
  hipFL.Close();

  // ==================================================================================================
  // Demo #3: Fully automated pulse control. Pulses both muscles at different rates for 30 seconds.
  // ==================================================================================================

  // set pulse rate for each muscle. SetPulseFrequency() sets dt_off automatically
  hipEX.SetPulseFrequency(30); //[Hz]
  hipFL.SetPulseFrequency(45); //[Hz]

  // pulse for 30 seconds
  Now = millis();             //[ms]
  while (millis()-Now < 30000){
    /*
     * Query the muscle objects to calculate elapsed time since last event and 
     *  determine whether the valve positions are correct. When an incorrect 
     *  valve position is encountered, the valve and valve position boolean are 
     *  both flipped and the associated timer is reset. This strategy avoids 
     *  latching the cursor and reduces supervision to the two checks shown below.
 
     */
    hipEX.ShouldPulseStart();
    hipFL.ShouldPulseStart();
    hipEX.ShouldPulseEnd();
    hipFL.ShouldPulseEnd();

    /*
     * NOTE: The functions <muscle>.ShouldPulsesStart() and <muscle>.ShouldPulseEnd() 
     *  are combined into <muscle>.Pulse_Nanny(), which allows access with a 
     *  single command, but here I am showing them separately, for demonstrative 
     *  purposes. 
     */
  }
  
  // ensure that the valves both end in the close position
  hipFL.Close();
  hipEX.Close();
}

void loop() { 
  //not used in this example
}
