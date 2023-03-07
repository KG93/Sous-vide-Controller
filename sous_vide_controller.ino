// EEPROM
#include <EEPROM.h>
const int EEPROM_ADDR_KP = 0; // memory addresses for PID parameters
const int EEPROM_ADDR_KI = 4;
const int EEPROM_ADDR_KD = 8;

// PID
#include <analogWrite.h>
#include <QuickPID.h> // the QuickPID library only supports autotuning until version 2.5

// thermal sensor
#include "Adafruit_MAX31865.h"
#include <pt100rtd.h>

/*
// Display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
*/

// Display
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c oled;
#define DisplayI2CAddress  0x3C


// Encoder
#include <TimerOne.h>
#include <ClickEncoder.h>

//unsigned int showMenuDuration = 4000; //time continue displaying the temperature setpoint after setpoint change in ms

//relay
#define relayPin  5
#define relayPin2  6
const unsigned int relayWindowTime = 500;
unsigned long windowStartTime;
unsigned long lastSwitchOnTime;

unsigned long timerInMinutes = 0;
unsigned long timedPIDStartTime;

//buttons
#define button1  7
#define button2  8
#define button3  9

//encoder
#define ENC1   2
#define ENC2   3
#define ENCButton   4
#define STEPS 2
uint8_t buttonState;

ClickEncoder encoder(ENC1, ENC2, ENCButton, STEPS);
int16_t last = 0, value = 0;
void timerIsr() 
{
  encoder.service();
  value += encoder.getValue();
}

//MAX31865 rtd board
#define MAXCLK  13
#define MAXDO   12
#define MAXDI   11
#define MAXCS   10
int maxTemp = 98;
int minTemp = 0;
float tempIncrement = 0.1;
#define RREF 430.0 //reference resistor in 0hm
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(MAXCS, MAXDI, MAXDO, MAXCLK);
pt100rtd rtdLookupTable;
unsigned long faultCheckTime;

// buzzer
#define BUZZERPIN   15

// PID
float temperatureSetpoint = 58;
int temperatureSetpointInt = temperatureSetpoint * 10;
float temperatureReading = 25, pidOutput = 0;
float maxOutput = relayWindowTime;

float defaultKp = 30; // default PID values
float defaultKi = 0.5;
float defaultKd = 0.1;

float Kp = defaultKp; // initialize PID values to default
float Ki = defaultKi;
float Kd = defaultKd;

float maxKp = 100; // define range for PID values to default & > 0
float maxKi = 30;
float maxKd = 10;

float KpFromEEPROM, KiFromEEPROM, KdFromEEPROM; // for the PID values from EEPROM

float POn = 1.0; // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0; // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0
bool error;
QuickPID myQuickPID = QuickPID(&temperatureReading, &pidOutput, &temperatureSetpoint,Kp, Ki, Kd, POn, DOn, QuickPID::DIRECT);

// AutotunePID
const uint32_t autotuneSampleTimeUs = 100000; // 100ms
float hysteresis = 0.5;
float autotuneOutput = maxOutput/3.0;  // 1/3 of range for symetrical waveform
float outputStep = autotuneOutput;

enum MenuState {
    PIDMenu,
    autoTuneMenu,
    timerMenu,
    pidParametersMenu,
};// the different menus
MenuState menuState = PIDMenu;

enum ChangeParameterState {
    changeKp = 0,
    changeKi,
    changeKd
};// the submenus for changing the PID values
ChangeParameterState changeParameterState = changeKp;
bool paramChangeable = false;

enum MachineState {
    menu = 0,
    PIDRoutine,
    autoTune,
    timedPIDRoutine,
}; // the different system states; whether durning pid routine or on menu, etc.
MachineState machineState = menu;

  
void setup() // set up the system initially
{
  //SPI.begin();
  Serial.begin(9600); //Start serial
  getStartTime();
  setUpPins();  
  setUpEncoder();  
  setUpMax31865();
  setUpDisplay();
  loadPID_FromEEPROM();
  setUpPID();
}

void loop() // the main loop
{
  readRDT(); // get the temperature
  processEncoderAndButtons(); // read the encoder and process the input
  updateDisplay(); // show system state on the display

  if (machineState == PIDRoutine || machineState == timedPIDRoutine) // PID routin (with timer)
  {  
    computePID();
    controlRelay();
  }
  else if (machineState == menu) // on menu
  {
    digitalWrite(relayPin, LOW);
  }
  else if (machineState == autoTune) // during the autotune routine for the PID parameters
  {
    if (myQuickPID.autoTune) // Avoid dereferencing nullptr after _myPID.clearAutoTune()
    {
      switch (myQuickPID.autoTune->autoTuneLoop()) 
      {
        case myQuickPID.autoTune->AUTOTUNE: // during the autotune loop
          autotuneRelay();
          break;
  
        case myQuickPID.autoTune->TUNINGS:  // once after tuning
          myQuickPID.autoTune->setAutoTuneConstants(&Kp, &Ki, &Kd); // set new tunings
          myQuickPID.SetMode(QuickPID::AUTOMATIC); // setup PID
          myQuickPID.SetSampleTimeUs(autotuneSampleTimeUs);
          myQuickPID.SetTunings(Kp, Ki, Kd, POn, DOn); // apply new tunings to PID
          oled.clear();
          oled.set1X();
          oled.setCursor(0, 0);
          oled.print("Kp ");
          oled.println(Kp);
          oled.print("Ki ");
          oled.println(Ki);
          oled.print("Kd ");
          oled.println(Kd);
          digitalWrite(relayPin, LOW);
          tone(BUZZERPIN, 400, 4000);
          delay(20000);
          savePID_toEEPROM();
          break;
  
        case myQuickPID.autoTune->CLR: // clear autotuner
          if (machineState != PIDRoutine) 
          {
            myQuickPID.clearAutoTune(); // releases memory used by AutoTune object
            machineState = PIDRoutine;
            oled.clear();
          }
          break;
      }
    }
  }
}

void processEncoderAndButtons()
{
  //value += encoder.getValue();

   buttonState = encoder.getButton();  
  if (machineState == menu) // show menu
  {
    if (buttonState == ClickEncoder::DoubleClicked) // switch to next menu
    {
      menuState = static_cast<MenuState> (menuState + 1);
      menuState = static_cast<MenuState> (menuState % 4);
      paramChangeable = false;
      oled.clear();
    }
    
    if (buttonState != 0 || value != last) // double click was checked earlier -> only single click and release remain
    {  
      switch (menuState) 
      {      
        case PIDMenu:
          temperatureSetpoint += (value - last) * tempIncrement; // update temperature setpoint      
          switch (buttonState) 
          {      
            case ClickEncoder::Released: // start PID routine      
              machineState = PIDRoutine;
              setUpPID();                
              oled.clear();
              break;
      
            case ClickEncoder::Clicked:       
              break;
          }
        break;
        
        case autoTuneMenu:
          temperatureSetpoint += (value - last) * tempIncrement; // update temperature setpoint    
          switch (buttonState) 
          {
            case ClickEncoder::Released: // start PID autotune routine       
              machineState = autoTune;
              windowStartTime = millis();
              setupAutoTune();
              oled.clear();
              break;
      
            case ClickEncoder::Clicked:      
              break;
          }         
        break;
        
        case timerMenu:          
          if(timerInMinutes + value - last < 0) // update timer value    
          {
            timerInMinutes = 0;
          }
          else
          {
            timerInMinutes += (value - last);
          }            
          switch (buttonState) 
          {
            case ClickEncoder::Released: // start timed PID routine         
              machineState = timedPIDRoutine;
              timedPIDStartTime = millis();
              setUpPID();
              oled.clear();
              break;
      
            case ClickEncoder::Clicked:       
              break;
          }        
        break;

        case pidParametersMenu: // menu for showing/changing the PID parameters
          if (paramChangeable) // PID parameters are changeable
          {
            switch (changeParameterState) 
            {      
              case changeKp: // update Kp value                
                 Kp = constrain(Kp + (value - last) * 0.1, 0, maxKp);
                 break;
        
              case changeKi: // update Ki value
                 Ki = constrain(Ki + (value - last) * 0.1, 0, maxKi);
                 break;
        
              case changeKd: // update Kd value
                Kd = constrain(Kd + (value - last) * 0.1, 0, maxKd);
                break;
            }        
          }
          switch (buttonState) // PID parameters are only shown
          {      
            case ClickEncoder::Released:
              if(paramChangeable)    
              {
                paramChangeable = false;
                savePID_toEEPROM();  
              }
              else
              {
                paramChangeable = true;
              }
              oled.clear();
              break;
      
            case ClickEncoder::Clicked: 
              changeParameterState = static_cast<ChangeParameterState> (changeParameterState + 1);
              changeParameterState = static_cast<ChangeParameterState> (changeParameterState % 3);
              oled.clear();      
              break;
          }
        break;
      }
    }
  }
  else // not showing menu; in PID/autotune routine
  {
    temperatureSetpoint += (value - last) * tempIncrement; // update temperature setpoint 
    if (buttonState != 0) 
    {
      switch (buttonState) // abort PID/autotune routine
          {      
            case ClickEncoder::Released:      
              machineState = menu;
              if(machineState == autoTune)
              {
                myQuickPID.clearAutoTune();
              }
              digitalWrite(relayPin, LOW);
              oled.clear();
              break;
          }
      }
  }
  last = value; 
  if(temperatureSetpoint > maxTemp)  // keep temperature setpoint in admissible range
  {
    temperatureSetpoint = maxTemp;
  }
  if(temperatureSetpoint < minTemp) 
  {
    temperatureSetpoint = minTemp;
  }
}

void readRDT() // read the thermocouple
{
  uint16_t rtd = max31865.readRTD();
  float ratio = rtd / 32768.0;

  // float c = max31865.temperature(100, RREF);
  float resistance = RREF * ratio;
  temperatureReading = rtdLookupTable.celsius(resistance);
  
  checkMax31865Faults();

  if (isnan(rtd)|| temperatureReading < -100 || temperatureReading > 800) 
  {
    error = true;
  } 
  if (error) 
  {
    Serial.println("Something wrong with thermocouple!");
    oled.setCursor(90, 0);
    oled.set1X();
    oled.println("error");
  }
}

void computePID()  // compute the PID output
{
  if (abs(temperatureSetpoint - temperatureReading) > 5)
  {
    if(myQuickPID.GetKi() != 0)
    {
      myQuickPID.SetTunings(Kp,0,Kd); // prevent integrator windup
    }
  }
  else
  {
    if (myQuickPID.GetKi() == 0)
    {
      myQuickPID.SetTunings(Kp,Ki,Kd);
    }
  }
  myQuickPID.Compute();  
}

void updateDisplay() // write the system state to the display
{
  if (machineState == menu) // on the menu
  {
    switch (menuState) {      
      case PIDMenu:
        oled.setCursor(0, 0);
        oled.set1X();
        oled.print(temperatureSetpoint); 
        oled.println("C");          
        oled.println("Hold to start");  
        oled.set2X();
        oled.println("PID");  
      break;
      
      case autoTuneMenu:
        oled.setCursor(0, 0);
        oled.set1X();
        oled.println(temperatureSetpoint);  
        oled.println("Hold to start");  
        oled.println("Autotune");  
      break;
      
      case timerMenu:
         oled.setCursor(0, 0);
        oled.set1X();
        oled.print(timerInMinutes);  
        oled.println(" minutes");  
        oled.println("Hold to start");  
        oled.println("timedPID");  
      break;
      
      case pidParametersMenu:
        oled.setCursor(0, 0);
        oled.set1X();
        if(paramChangeable)
        {
          oled.println("Change Param");
          if(changeParameterState == changeKp)
          {
            oled.print("Kp ");  
            oled.println(Kp);            
            oled.print("Default "); 
            oled.println(defaultKp);
            oled.print("EEPROM "); 
            oled.println(KpFromEEPROM);                       
          }
          if(changeParameterState == changeKi)
          {
            oled.print("Ki ");  
            oled.println(Ki);
            oled.print("Default "); 
            oled.println(defaultKi);
            oled.print("EEPROM "); 
            oled.println(KiFromEEPROM);                         
          } 
          if(changeParameterState == changeKd)
          {
            oled.print("Kd ");  
            oled.println(Kd); 
            oled.print("Default "); 
            oled.println(defaultKd);
            oled.print("EEPROM "); 
            oled.println(KdFromEEPROM);                                    
          }
        }
        else
        {
          oled.println("PID Param");
          oled.print("Kp ");  
          oled.println(Kp);  
          oled.print("Ki ");  
          oled.println(Ki);  
          oled.print("Kd ");  
          oled.println(Kd);  
        }
        
      break;
    }
    if(!error)
    {
      oled.setCursor(90, 0);
      oled.set1X();
      oled.println(temperatureReading);
    }
  }
  else  
  {  
    oled.setCursor(0, 0);
    oled.set1X();
    oled.println(temperatureSetpoint);  
    oled.set2X();
    oled.println(temperatureReading);
    oled.set1X();
    if(machineState == autoTune)
    {
      oled.println("Autotune");
    }
    if(machineState == PIDRoutine)
    {
      oled.println("PID");
    }  
    if(machineState == timedPIDRoutine)
    {      
      oled.print(timerInMinutes - (millis() - timedPIDStartTime)/60000.0); // print timer
      oled.println(" minutes");  
    }
  }
}

void controlRelay()
{
  if (millis() - windowStartTime > relayWindowTime)
  { //time to shift the Relay Window
    windowStartTime += relayWindowTime;
  }
  if (machineState == timedPIDRoutine && (millis() - timedPIDStartTime)/60000.0 > timerInMinutes) // turn relay off after timer has elapsed
  {
    if(myQuickPID.GetMode() == QuickPID::AUTOMATIC)
    {
      myQuickPID.SetMode(QuickPID::MANUAL);
    }
    digitalWrite(relayPin, LOW);
    tone(BUZZERPIN, 400, 8000);
    machineState = menu;
    oled.clear();
    return;  
  }

  //Serial.print("PID = ");
  //Serial.println(pidOutput);
  //Serial.print("time in interval = ");  
  //Serial.println(millis() - windowStartTime)
  
  if(error) //in case of error -> relay swich off
  {
    digitalWrite(relayPin, LOW);
    return;
  }
  if(temperatureReading > maxTemp)
  {
    digitalWrite(relayPin, LOW);
    return;
  }
  if(temperatureReading < temperatureSetpoint - 8) // full power till setpoint - 8 degrees
  {
    digitalWrite(relayPin, HIGH);
    return;
  }
  if((pidOutput > (millis() - windowStartTime) && temperatureReading < temperatureSetpoint + 1))
  {
    digitalWrite(relayPin, HIGH);
  }
  else
  {
    digitalWrite(relayPin, LOW);
  }
}

void autotuneRelay()
{
  if (millis() - windowStartTime > relayWindowTime)
  { //time to shift the Relay Window
    windowStartTime += relayWindowTime;
  }
  if (error) //in case of error -> relay swich off
  {
    digitalWrite(relayPin, LOW);
    return;
  }
  if(temperatureReading > maxTemp)
  {
    digitalWrite(relayPin, LOW);
    return;
  }
  if (millis() - windowStartTime < pidOutput)
  {
    digitalWrite(relayPin, HIGH);
  }
  else
  {
    digitalWrite(relayPin, LOW);
  }  
}

void getStartTime() 
{
  lastSwitchOnTime = millis();
}

void setUpPins()
{
  pinMode(ENC1, INPUT_PULLUP);
  pinMode(ENC1, INPUT_PULLUP);
  pinMode(ENCButton, INPUT_PULLUP);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  
  pinMode(relayPin, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(BUZZERPIN, OUTPUT);
}

void setUpEncoder()
{
  Timer1.initialize(100);
  Timer1.attachInterrupt(timerIsr); 
  encoder.setAccelerationEnabled(true);
  encoder.setDoubleClickEnabled(true);
}

void setUpDisplay()
{
  oled.begin(&Adafruit128x64, DisplayI2CAddress);
  oled.setFont(TimesNewRoman16_bold);
  oled.clear();
}

void setUpMax31865()
{
  delay(700);
  //max31865.begin(MAX31865_2WIRE);
  //max31865.begin(MAX31865_3WIRE);
  max31865.begin(MAX31865_4WIRE);
  delay(700);
//  readRDT();

}

void checkMax31865Faults()
{
  if (faultCheckTime == windowStartTime && machineState != menu) //fault check only once per relay window in PID mode
  {
    return;
  }
  uint8_t fault = max31865.readFault();
  if (fault) 
  {
    error = true;

    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    
    if (fault & MAX31865_FAULT_HIGHTHRESH) 
    {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) 
    {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) 
    {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) 
    {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) 
    {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) 
    {
      Serial.println("Under/Over voltage");
    }   
    max31865.clearFault();
  }
  else
  {
    error = false;
  }
  faultCheckTime = windowStartTime; //update faultCheckTime, so that function runs only once during a relay window
}

void loadPID_FromEEPROM()
{
  // get the PID parameters from memory
  EEPROM.get(EEPROM_ADDR_KP, KpFromEEPROM);
  EEPROM.get(EEPROM_ADDR_KI, KiFromEEPROM);
  EEPROM.get(EEPROM_ADDR_KD, KdFromEEPROM);
  // check if the PID parameters from memory differ from the default values
  if (KpFromEEPROM != defaultKp || KiFromEEPROM != defaultKi || KdFromEEPROM != defaultKd)
  {
    Kp = KpFromEEPROM;
    Ki = KiFromEEPROM;
    Kd = KdFromEEPROM;
  }
  
  // Check if the PID parameters are within the expected range or if they are NaNs
  if (Kp < 0 || Kp > maxKp || Ki < 0 || Ki > maxKi || Kd < 0 || Kd > maxKd || isnan(KpFromEEPROM) || isnan(KiFromEEPROM) || isnan(KdFromEEPROM))   
  {
    Kp = defaultKp;
    Ki = defaultKi;
    Kd = defaultKd;

    // Write default parameters to EEPROM
    EEPROM.put(EEPROM_ADDR_KP, defaultKp);
    EEPROM.put(EEPROM_ADDR_KI, defaultKi);
    EEPROM.put(EEPROM_ADDR_KD, defaultKd);
    
    // Prompt the user to enter the PID parameters manually
    // oled.clear();
    // oled.println("Enter PID parameters:");
    // oled.println("Kp (0-100):");
    // Wait for the user to input a value for Kp using the encoder and buttons...
    // oled.println("Ki (0-10):");
    // Wait for the user to input a value for Ki using the encoder and buttons...
    // oled.println("Kd (0-10):");
    // Wait for the user to input a value for Kd using the encoder and buttons...
    oled.clear();
    oled.set1X();
    oled.println("Invalid PID parameters");
    oled.println("Restoring default parameters");
    tone(BUZZERPIN, 400, 4000);
    delay(2000);
  }
  else
  {
    oled.clear();
    oled.set1X();
    oled.println("Loaded PID parameters from EEPROM");
    oled.println(Kp);
    oled.println(Ki);
    oled.println(Kd);
    delay(4000);
    oled.clear();
  }
}

void savePID_toEEPROM() // make the PID valus persistent to the next reboot
{
  EEPROM.put(EEPROM_ADDR_KP, Kp);
  EEPROM.put(EEPROM_ADDR_KI, Ki);
  EEPROM.put(EEPROM_ADDR_KD, Kd);
  oled.clear();
  oled.set1X();
  oled.println("Saving PID");
  oled.println("parameters to");
  oled.println("eeprom");
  delay(4000);
  oled.clear();
  KpFromEEPROM = Kp;
  KiFromEEPROM = Ki;
  KdFromEEPROM = Kd;
}

void setUpPID()
{
  windowStartTime = millis();
  myQuickPID.SetOutputLimits(0, maxOutput);
  myQuickPID.SetMode(QuickPID::AUTOMATIC);  
}

void setupAutoTune() 
{
  setUpPID();

  // Select a tuning algorithm.
  //myQuickPID.AutoTune(tuningMethod::ZIEGLER_NICHOLS_PI);
  //myQuickPID.AutoTune(tuningMethod::ZIEGLER_NICHOLS_PID);
  //myQuickPID.AutoTune(tuningMethod::TYREUS_LUYBEN_PI);
  //myQuickPID.AutoTune(tuningMethod::TYREUS_LUYBEN_PID);
  myQuickPID.AutoTune(tuningMethod::CIANCONE_MARLIN_PI); // other algorithms created large Kd values during my testing and led to unstable temperatures
  //myQuickPID.AutoTune(tuningMethod::CIANCONE_MARLIN_PID);
  //myQuickPID.AutoTune(tuningMethod::AMIGOF_PID);
  //myQuickPID.AutoTune(tuningMethod::PESSEN_INTEGRAL_PID);
  //myQuickPID.AutoTune(tuningMethod::SOME_OVERSHOOT_PID);
  //myQuickPID.AutoTune(tuningMethod::NO_OVERSHOOT_PID);

  myQuickPID.autoTune->autoTuneConfig(outputStep, hysteresis, temperatureSetpoint, autotuneOutput, QuickPID::DIRECT, 0, autotuneSampleTimeUs);  
  if (constrain(autotuneOutput, 0, maxOutput - outputStep - 5) < autotuneOutput) 
  {
    Serial.println(F("AutoTune test exceeds outMax limit. Check output, hysteresis and outputStep values"));
    machineState = menu;
  }
}
