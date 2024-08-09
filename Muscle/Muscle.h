#ifndef MY_MUSCLE_H
#define MY_MUSCLE_H

#include <Arduino.h>

class Muscle {
  private:
    int valve_pin;
    int pressure_pin;
    
  public:
    //initialize the class object      
    Muscle(String Name, int valve_pin, int pressure_pin);

    //initializer function
    void begin();

    //pressure functions
    void updatePressure();
    float getPressure();
    float getPressureGradient();

    //valve control functions
    void Open();
    void Close();

    //pulse control functions
    bool ShouldPulseEnd();
    bool ShouldPulseStart();
    void thisPulseStart();
    void thisPulseEnd();
    void Pulse_Nanny();
    void SetPulseFrequency(float);

    //=============================MUSCLE ID PARAMETERS==================================
    String Name;

    //============================PRESSURE TRACKING PARAMETERS===========================
    float PRESSURE_NOW;                     //most recent averaged pressure from the sensor
    float PRESSURE_PREVIOUS;                //previous averaged pressure, assigned at update
    float PRESSURE_GRADIENT;                //defined by pressure_now - pressure_prev
    
    //==============================PULSE CONTROL PARAMETERS=============================

    // TOGGLES
    bool IS_PULSING_ENABLED;                //toggle whether this muscle should be pulsing
    bool IS_A_PULSE_ACTIVE;                 //(during pulsing) tracks whether the valve is open  

    // TIME TRACKING VARIABLES
    unsigned long WHEN_THIS_PULSE_STARTED;  //when the valve last opened 
    unsigned long WHEN_LAST_PULSE_ENDED;    //when the valve last closed 

    // 
    /* ================== PULSE CONTROL PARAMETERS ===========================================
     * DT_ON and DT_OFF are elements of the DUTY_CYCLE. The relationship to PULSE_FREQUENCY is 
     * 
     *          DUTY_CYCLE = DT_ON + DT_OFF = 1/PULSE_FREQUENCY
     *          
     * DT_ON is a fixed value and DT_OFF is derived from the user-set PULSE_FREQUENCY when the 
     * SetFrequency() function is called. DUTY_CYCLE is redundant and unused in the code. 
     */
    float PULSE_FREQUENCY;
    float DT_ON; 
    float DT_OFF;         
  
};

#endif
