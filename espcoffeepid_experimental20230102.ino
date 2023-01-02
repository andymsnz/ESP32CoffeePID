// ## PID controller for a single boiler esprsso machine for ESP32 ##
// https://github.com/andymsnz/ESP32CoffeePID

// Hardware used in reference design //////////////////////////////////////////////////////

// ESP32 Devkit board
// 2.0" TFT 320x240 pixels SPI screen (ST7789V driver)
// 40A DA Solid state relay (boiler control)
// MAX6675 Thermocouple
// 1.2Mpa XDB-301 pressure sensor 0.5-4.5v single signal wire
// TOF10120 distance sensor
// WS2812 addressable LED strip
// 110-220v AC - DC optocoupler for brew-switch detection
// Zero Crossing AC Dimmer module (RobotDyn)

// Feature Log /////////////////////////////////////////////////////////////////////////////

// Functioning & Tested features //////////////////////////////////////////////////////////
// ## Rudimentary web interface ##
// ## PID autotune capability for boiler PID ##
// ## Persistent memory for tuning variable (SPIFFS) ##
// ## Pump pressure reading (XDB-301) ##
// ## WS2812 LED Tank status lighting (FastLED) ##
// ## Reservoir water level sensing (TOF10120) ## REQUIRES TESTING

// ## Features in development ##

// ## PID target pressure control , utlising ZeroCrossing AC Dimmer module ##
// ## Pump PID Autotune

// Function quick reference ///////////////////////////////////////////////////////////////

// appendToSystemLog() // Writes to a SPIFFS log file ## REQUIRES TESTING ##
// click() // Handle button events
// BrewSwitch() // Checks if the opticoupler has been triggered by the brew switch ## REQUIRES TESTING ##
// CheckButtons(); //Check button handlers
// CheckEncoders(); //Check the rotary encoder
// ReadPumpPressure() // Reads and converts the XDB301 pressure sensor ## REQUIRES TESTING ##
// ReadBoilerTemp() // Read the MAX6675 k-type boiler thermocouple
// ReadTOF() // Reads the distance from the TOF10120 time of flight water level sensor
// TankLights() // Set's the above water reservoir lights to colour and state

// ## PID & Autotune PID ##
// AutoTuneHelper() // Recovers and sets PID mode forautotune
// changeAutoTune() // 
// DoModel() // Model PID autotune in simulation
// DriveRelay() // Emulated PWM for the SSR controlling the boiler
// PIDcycle() // Main PID autotuen function

// ## SPIFFS file management ##
// store_setpoint() // Write persistent values to SPIFFS file
// read_setpoint() // Read persistent value from SPIFFS file

// Libraries //////////////////////////////////////////////////////////////////////////////

#include <Button2.h> // Button handler
#include <FastLED.h>  // WS2812 | Neopixel 
#include <max6675.h> // Thermocouple library
#include <PID_v1.h> // PID library
#include <PID_AutoTune_v0.h> // PID autotuning library
#include <PNGdec.h> // PNG image decode
#include <RBDdimmer.h> // Library for RobotDyn AC Dimmer module
#include <SPI.h> //SPI library
#include "SPIFFS.h" // Add SPIFFS library
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <WebServer.h> // ESP32 web server library
#include <WiFiUdp.h> // UDP library
#include <WiFi.h> // WiFi library
#include <WiFiAP.h> // Wifi AP 
#include <Wire.h> // I2C handing

// END OF LIBRARIES ///////////////////////////////////////////////////////////////////////

// Define ESP32 GPIO Pins (Future WIP, move to pins.h header) /////////////////////////////

//Brew Switch (not deployed)
#define BREW_SW_PIN 19
// Buttons (if used)
#define BUTTON_A_PIN  34 // set pins as required
#define BUTTON_B_PIN  35 // set pins as required
// //Rotary encoder
// #define ENCODER_A_PIN 2
// #define ENCODER_B_PIN 4
// #define ENCODER_SW_PIN 27
//I2C Bus  
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
// #define TOF10120_ADDRESS 0x54 // I2C address of the TOF10120 sensor
// #define TOF10120_ADDRESS 0x52 // I2C address of the TOF10120 sensor
// #define TOF10120_DISTANCE_REG 0x00 // Register addresses for the TOF10120 sensor
//MAX6675 Thermocouple
#define TC_SO_PIN 12 // MISO=Serial Out
#define TC_CS_PIN 15 // CS = chip select CS pin
#define TC_SCK_PIN 14 // SCK = Serial Clock pin
#define TC_SI_PIN 13 // MOSI // SI PIN NOT USED FOR MAX6675
//Neopixel | LED Pins
#define NEOPIX_PIN 27
#define NUM_LEDS 5 // How may LEDs on the PIN
//Pump AC Dimmer
#define PUMP_ZC_PIN 26
#define PUMP_PWM_PIN 25
//Pressure Sensor
#define PRESSURE_PIN 33
//Solid state relay (SSR)
#define RELAY_PIN 32 //Boiler SSR Control pin (HIGH 3v3)
// SPI SCREEN (SET IN TFT_eSPI Listed here for reference only) || ST7789V 2.0" RGB TFT 320x240 screen specified in this build
// SCL | GPIO 18 black
// SDA | GPIO 23 black
// RST | GPIO 16 yellow 
// DC | GPIO 17 white 
// CS | GPIO 5 red

// END OF DEFINITIONS /////////////////////////////////////////////////////////////////////


// Variables //////////////////////////////////////////////////////////////////////////////

// Wifi AP details SSID and IP Config
const char* ssid = "ESP Coffee PID"; // Set the name of the AP
const char* password = "";  // Set the password for the AP (leave blank for open AP)
IPAddress local_IP(192, 168, 1, 1);  // Set the host IP for the AP
IPAddress gateway(192, 168, 1, 1);   // Set the gateway IP
IPAddress subnet(255, 255, 255, 0);  // Set the subnet mask

// Image arrays
#include "backgroundloading.h" // Startup boot screen image
#include "infopageb.h" //main page backdrop
#define MAX_IMAGE_WDITH 320 // Adjust for your images
int16_t xpos = 0;
int16_t ypos = 0;

// PID Variables Boiler //////////////////////////////////////////////////////////////////

// PID gains (keep a history until it's tuned)
double Kp = 2.78, Ki = 0.05, Kd = 0; // (result of earlier autotune)
double input = 0, output = 0, setpoint = 95; // Input, output, setpoint
double controltype = 1; //Set 0 for PI control and 1 for PID control autotuning parameters

// PID Autotune 
double kpmodel=1.5, taup=100, theta[50];
double OutputStart=5;
float tempcheckboiler;
byte ATuneModeRemember=2;

//set aTuneStep to ensure noticeable heating and cooling from aTuneStartValue
double aTuneStep=750, aTuneNoise=1, aTuneStartValue=750; 

// How far back to look back to find maxima and minima.
// For Slow processes this will be large, for fast processes, this will be smaller.
unsigned int aTuneLookBack=200; 

unsigned long  modelTime, serialTime;

// Relay response PID
int WindowSize = 1000;
unsigned long windowStartTime;

// Set to false to connect to the real world
bool useSimulation = false; //Keep false unless simulating

// PID Tuning?
bool tuning = false; //Keep False

// PID Variables PUMP /////////////////////////////////////////////////////////////////////

// PID Variable PUMP Pressure
double pumpKp = 1.0, pumpKi = 0.1, pumpKd = 0.01; // (result of earlier autotune)
double pumpinput = 0, pumpoutput = 0, pumpsetpoint = 9; // Input, output, setpoint
double pumpcontroltype = 1; //Set 0 for PI control and 1 for PID control autotuning parameters

// PID PUMP Autotune 
double pumpkpmodel=1.5, pumptaup=100, pumptheta[50];
double pumpOutputStart=5;
float pumpCheckPressure;
byte pumpATuneModeRemember=2;

//set aTuneStep to ensure noticeable pressure adjustment from aTuneStartValue
double pumpaTuneStep=750, pumpaTuneNoise=1, pumpaTuneStartValue=750; 

// How far back to look back to find maxima and minima.
// For Slow processes this will be large, for fast processes, this will be smaller.
unsigned int pumpaTuneLookBack=200; 
unsigned long  pumpmodelTime, pumpserialTime;

// Relay response PID
int pumpWindowSize = 200;
unsigned long pumpwindowStartTime;

// Set to false to connect to the real world
bool pumpuseSimulation = false; //Keep false unless simulating

// PID Tuning?
bool pumptuning = false; //Keep False


// SCREEN /////////////////////////////////////////////////////////////////////////////////

// TFT Charting
double temperatures[120]; // Array to store the temperature values
int currentIndex = 0; // Current index in the array

// Set the screen size and position of the graph
int screenWidth = 320;
int screenHeight = 240;
int graphX = 0;
int graphY = 0;
int graphWidth = 320;
int graphHeight = 240;

bool drawit = false; // main loop control for flipping to a temperature chart
double temperature = 0;
double setline = 0;

// Start time
unsigned long startTime; // A variable to store the start time of the system

// Pump pressure sensing XDB-301 1.2Mpa 0.5-4.5V linear senor
double xdbValue;
double PumpPressureBar;
double PumpPressureMpa;

// Brew switch detection
bool Brewing = false; // This is toggled when optocoupler detects brew switch has been turned on

// Thermocouple reading
int minReadTime = 250;

// Water reservoir level measurement using TOF10120
int secs = 0; 
float setSensorValue = 0; 

// for TOF10120 Laser Rangefinder Sensor
unsigned char ok_flag;
unsigned char fail_flag;
unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;
 
int x_mm; // distance in millimeters
float f_value; 

float WaterLevelMax = 160;
float WaterLevelNow;
int TankLevelPercentage;

// TOF END

//RBD Dimmer pump control
// 70 Commands ~14Bar
// 60 Commands ~10Bar
// 55 commands ~9Bar
// 45 commands ~5Bar
// 40 commands ~4Bar
int outVal = 57; // Start duty cycle for Pump (on AC Dimmer)

//RBB Dimmer pump END

// END OF VARIABLES //////////////////////////////////////////////////////////////////////////////

// Object instances

Button2 buttonA = Button2(BUTTON_A_PIN); // physical button at pre-defined pin
Button2 buttonB = Button2(BUTTON_B_PIN); // physical button at pre-defined pin
dimmerLamp pump(PUMP_PWM_PIN, PUMP_ZC_PIN); //initialase port for AC dimmer controlling the pump | zero crossing mode
MAX6675 thermocouple(TC_SCK_PIN, TC_CS_PIN, TC_SO_PIN);// create instance object of MAX6675
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // Create Boiler PID object
PID_ATune aTune(&input, &output);
PID pumpPID(&pumpinput, &pumpoutput, &pumpsetpoint, pumpKp, pumpKi, pumpKd, DIRECT); // Create PUMP PID object
PID_ATune pumpaTune(&pumpinput, &pumpoutput);
PNG png; // PNG decoder inatance
TFT_eSPI tft = TFT_eSPI(); // Define the tft object
WebServer server(80); // Create a web server on port
CRGB leds[NUM_LEDS]; // Declare an array to hold the LED colors

// END OF OBJECTS ////////////////////////////////////////////////////////////////////////////////


// Setup Void ////////////////////////////////////////////////////////////////////////////////////

void setup(void) {
 
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // Initialize the I2C bus
  
  Serial.begin(9600);// initialize serial monitor
  Serial.print("Serial started 9600");
  
  FastLED.addLeds<WS2812, NEOPIX_PIN, GRB>(leds, NUM_LEDS); // Initialize the FastLED object
  Serial.print("LEDs SETUP");

  pump.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE)
  pump.setPower(outVal);
  Serial.print("Pump Controller Online");

// Initialise PID and Autotuning
  windowStartTime = millis();

  pinMode(RELAY_PIN, OUTPUT); //Set relay pin as output
   if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=OutputStart;
    }
    modelTime = 0;
  }
 
  myPID.SetOutputLimits(0, WindowSize); //tell the PID to range between 0 and the full window size
  Serial.print("PID output set");
  myPID.SetMode(AUTOMATIC); // Initialize PID controller
  Serial.print("PID mode set");

   if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  aTune.SetControlType(controltype); //Set 0 for PI control and 1 for PID control autotuning parameters
  
  // Initialize TFT display 
  tft.init(); // Initialize display
  tft.setRotation(1); // Set rotation to landscape
  tft.fillScreen(TFT_BLACK); // Clear screen
  bootimagenew(); //load the bootscreen
  delay(1500);
  Serial.print("TFT bootup and wait");

  // Setup Wifi Accesspoint
  WiFi.mode(WIFI_AP);  // Set the ESP32 to AP mode
  WiFi.softAPConfig(local_IP, gateway, subnet);  // Configure the AP with the specified host IP, gateway, and subnet mask
  WiFi.softAP(ssid, password);  // Start the AP with the specified name and password
  Serial.print("Wifi Setup");

  tft.fillScreen(TFT_BLACK); // Clear screen
  // infopagedraw(); // draw background image
  Serial.print("Page Draw");
  read_setpoint();// Recover the past value for setpoint
  serverwebpages(); //setup webserver browser locations
  server.begin(); // Start web server
  Serial.print("Webserver UP");

  // button handler
  buttonA.setClickHandler(click);
  buttonB.setClickHandler(click);

}

///////////////////////////////////////////////////////////////////////////////////////////
// Main loop //////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  WaterTank(); // Check the level of the water tank
  ReadSensors(); // Check the boiler temp and pump pressur
  PumpControl();// Set the pump pressure control
  TankLights(); //Manage water tank lighting
  CheckButtons(); //Check button handlers
  // CheckEncoders(); //Check the rotary encoder

  //  Boiler Temperature management
  PIDcycle();  // PID autotune routine
  myPID.Compute();  // Calculate PID output
  RelayDrive();  // Drive the relay based on the PID calculation
  
  // Brewing & Pump pressure management
  BrewSwitch(); // Check if we are brewing 
  
  // DrawHomeScreen(); // Draw UI on main TFT - UNCOMMENT TO SHOW REGULAR UI

  // TEMPORARY SENSOR TEST
  // String PumpCurrentState = pump.getState();
  // String PumpCurrentPower = pump.getPower();

  tft.setTextColor(TFT_WHITE,TFT_BLACK); //Set text colour
  tft.setTextSize(2); // Set text size
  tft.setCursor(0,0); // Set first line position
  tft.println("Set point: " + (String(setpoint)) + " C");
  tft.println("Boiler Temp: " + (String(input)) + " C"); // thermo
  tft.println("Boiler Control: "+ (String(output / 10)) +" %"); // PID output
  tft.println("Presure Sensor: " + (String(xdbValue)) + "mV"); //RAW pressure signal
  tft.println("Pump Pressure: " + (String(PumpPressureBar)) + " bar"); //Bar pressure signal
  tft.println("Brew Switch State: " + (String(Brewing))); // Brew Switch State
  tft.println("Water Level: " + (String(TankLevelPercentage)) + " %"); //TOF10120 attempt
  tft.println("Pump State: " + String((pump.getState() ))); // Read the Pump State
  tft.println("Pump Power: " + String((pump.getPower() ))); // Read the Pump dimmer level
  server.handleClient(); // Handle client requests from the webserver
  Serial.print("LOOP");
}

///////////////////////////////////////////////////////////////////////////////////////////
// Water Reservoir Level Reading using TOF10120 ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void WaterTank(){
  //Time of flight Sensor (Water Tank Level - TOF10120)
  float TankAirGap = ReadDistance();
  WaterLevelNow = (WaterLevelMax - TankAirGap);
  TankLevelPercentage = ((WaterLevelNow / WaterLevelMax) * 100); // Calculate tank fill level
  // END time of flight
}

void TOF10120()
{
    for(int tof = 0; tof <=  9; tof++)
    {
      x_mm = x_mm + ReadDistance();
     
    }
       f_value = x_mm / 10;
       //Serial.print(f_value);
       //Serial.println(" mm");
       x_mm = 0;

     if ( (f_value < setSensorValue) || ( f_value > setSensorValue))
     {
      if ( secs == 4 )
      {
        setSensorValue = f_value; 
      }

     }

     if ( setSensorValue == f_value )
     {
      secs = 0;
     }
    
 Serial.print("Distance in mm: "); 
 Serial.println(setSensorValue);
}


int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}
 


void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(0x52); // transmit to device #82 (0x52), you can also find this address using the i2c_scanner code, which is available on electroniclinic.com
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(0x52, cnt);    // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }
}
 
int ReadDistance(){
    SensorRead(0x00,i2c_rx_buf,2);
    lenth_val=i2c_rx_buf[0];
    lenth_val=lenth_val<<8;
    lenth_val|=i2c_rx_buf[1];
    delay(100); 
    return lenth_val;
}


///////////////////////////////////////////////////////////////////////////////////////////
// Core Sensor Reading ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

// Read the thermocouple attached to the boiler and the pump pressure sensor
void ReadSensors(){

  //Thermocouple
  input = thermocouple.readCelsius();   // Read boiler temperature from thermocouple
  delay(minReadTime); // Pause for MAX6675 minimum update cadence (no less than 250ms

  //Pump pressure
   xdbValue = analogRead(PRESSURE_PIN); // Read the value from the XDB 1.2Mpa pressure sensor
   PumpPressureMpa = ((xdbValue - 500) / 4000 * 1.2); // Convert voltage to Mpa for linear sensor as per spec sheet
   PumpPressureBar = (PumpPressureMpa * 10); // Convert Mpa to Bar

}

///////////////////////////////////////////////////////////////////////////////////////////
// Control in-Tank LED addresable status array ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void TankLights() {
  
  // Check water level
  if (TankLevelPercentage < 25) {
    // Set all the LEDs to blue
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].setRGB(0, 0, 255);
    }
    FastLED.show();
    return; // Exit the function
  }

  // Check if the system has been running for 15 minutes or more
  if (millis() - startTime >= 900000) {
    // Set all the LEDs to green
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].setRGB(0, 255, 0);
    }
  }
  // Check if the temperature is over minimum temp for minimum time ##This is to be tuned, not sure if 2 mins at target is enough to heatsoak the machine for a shot##
  else if (temperature > (setpoint - 5)) {
    static unsigned long temperatureStartTime = millis(); // A variable to store the start time of the high temperature
    if (millis() - temperatureStartTime >= 120000) {
      // Set all the LEDs to GREEN if the temperature has been 5 degrees below or above setpoint for 2 minutes or more
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].setRGB(0, 255, 0);
      }
    }
  }
  // If none of the above conditions are met, set the LEDs to red
  else {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].setRGB(255, 0, 0);
    }
  }

  // Update the LEDs
  FastLED.show();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Boiler PID Controller //////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void DoModel()
{
  // Cycle the dead time
  for(byte i=0;i<49;i++)
    {
      theta[i] = theta[i+1];
    }
    // Compute the Input
    input = (kpmodel / taup) *(theta[0]-OutputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the Output to the desired starting frequency.
    aTuneStartValue = output; //KP Initial aTuneStartValue will be = output at Toggle
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(bool start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


// Main PID & Autotune function

void PIDcycle() {
  
unsigned long now = millis(); 
 if(!useSimulation)
  { //pull the Input in from the real world
    tempcheckboiler = thermocouple.readCelsius();
   //Pass to Input of PID
   input = thermocouple.readCelsius();
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();
      myPID.SetTunings(Kp,Ki,Kd);
      AutoTuneHelper(false);
      store_setpoint();
    }
  }
   if(myPID.GetMode()==AUTOMATIC && tempcheckboiler >= (100)) //if the PID is ON and the boiler temperature is higher than the maximum
      {
        myPID.SetMode(MANUAL);//turn the PID off
        output=0; //turns off the boiler
      }  

      if(myPID.GetMode()==MANUAL && tempcheckboiler < 80) //if the PID is OFF and the boiler temperature lower than the maximum -20 °C
      {
        myPID.SetMode(AUTOMATIC);//turn the PID on
      }  
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else // This is optimized for Relay output.
  { 
  if(now - windowStartTime>WindowSize)
  { // Time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(output > now - windowStartTime) digitalWrite(RELAY_PIN,HIGH);
  else digitalWrite(RELAY_PIN,LOW);  
}

  // Write the output status variable to the TFT (update later to on/off in loop above)
    tft.setCursor(118,175); // Set first line position
    tft.setTextColor(TFT_GREEN,TFT_BLACK);
    tft.setTextSize(2); // Set text size
    tft.println(output);
  
}

void RelayDrive(){

  // turn the output pin on/off based on pid output (simulated slow PWM)

  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)    //time to shift the Relay Window
    { 
    windowStartTime += WindowSize;
    }
  if(output > now - windowStartTime) digitalWrite(RELAY_PIN,HIGH);
  else digitalWrite(RELAY_PIN,LOW);

  double controlfraction = ((output / WindowSize) * 100);

    // tft.setCursor(118,175); //Set first line position
    // tft.setTextColor(TFT_GREEN,TFT_BLACK);
    // tft.setTextSize(2); //Set text size
    // // tft.println(output);
    // tft.println(String(controlfraction) + "%");

}

///////////////////////////////////////////////////////////////////////////////////////////
// PUMP DIRECT CONTROL PWM ////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void PumpControl(){
// Control the pump

  int preVal = outVal;
  pump.setPower(outVal); // setPower(0-100%);

}

///////////////////////////////////////////////////////////////////////////////////////////
// Pump PID controller ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void pumpDoModel()
{
  // Cycle the dead time
  for(byte i=0;i<49;i++)
    {
      pumptheta[i] = pumptheta[i+1];
    }
    // Compute the Input
    pumpinput = (pumpkpmodel / pumptaup) *(pumptheta[0]-pumpOutputStart) + pumpinput*(1-1/pumptaup) + ((float)random(-10,10))/100;

}

void pumpchangeAutoTune()
{
 if(!pumptuning)
  {
    //Set the Output to the desired starting frequency.
    pumpaTuneStartValue = pumpoutput; //KP Initial aTuneStartValue will be = output at Toggle
    pumpaTune.SetNoiseBand(pumpaTuneNoise);
    pumpaTune.SetOutputStep(pumpaTuneStep);
    pumpaTune.SetLookbackSec((int)pumpaTuneLookBack);
    pumpAutoTuneHelper(true);
    pumptuning = true;
  }
  else
  { //cancel autotune
    pumpaTune.Cancel();
    pumptuning = false;
    pumpAutoTuneHelper(false);
  }
}

void pumpAutoTuneHelper(bool start)
{
  if(start)
    pumpATuneModeRemember = pumpPID.GetMode();
  else
    pumpPID.SetMode(ATuneModeRemember);
}


// Main PID & Autotune function

void pumpPIDcycle() {
  
unsigned long now = millis(); 
 if(!pumpuseSimulation)
  { //pull the Input in from the real world
    ReadSensors();
    pumpCheckPressure = PumpPressureBar;
   //Pass to Input of PID
   pumpinput = PumpPressureBar;
  }
  
  if(pumptuning)
  {
    byte val = (pumpaTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!pumptuning)
    { //we're done, set the tuning parameters
      pumpKp = pumpaTune.GetKp();
      pumpKi = pumpaTune.GetKi();
      pumpKd = pumpaTune.GetKd();
      pumpPID.SetTunings(pumpKp,pumpKi,pumpKd);
      pumpAutoTuneHelper(false);
      store_setpoint();
    }
  }
   if(pumpPID.GetMode()==AUTOMATIC && pumpCheckPressure >= (15)) //if the PID is ON and the boiler temperature is higher than the maximum
      {
        pumpPID.SetMode(MANUAL);//turn the PID off
        pumpoutput=0; //turns off the pump
      }  

      if(pumpPID.GetMode()==MANUAL && pumpCheckPressure < 0) //if the PID is OFF and the boiler temperature lower than the maximum -20 °C
      {
        pumpPID.SetMode(AUTOMATIC);//turn the PID on
      }  
  else pumpPID.Compute();
  
  if(pumpuseSimulation)
  {
    pumptheta[30]=pumpoutput;
    if(now>=pumpmodelTime)
    {
      pumpmodelTime +=100; 
      pumpDoModel();
    }
  }
  else // This is optimized for Relay output.
  { 
  if(now - pumpwindowStartTime>pumpWindowSize)
  { // Time to shift the Relay Window
    pumpwindowStartTime += pumpWindowSize;
  }
  if(pumpoutput > now - pumpwindowStartTime) pump.setPower(pumpoutput);
  // else digitalWrite(RELAY_PIN,LOW);  
}

  // Write the output status variable to the TFT (update later to on/off in loop above)
    tft.setCursor(118,175); // Set first line position
    tft.setTextColor(TFT_GREEN,TFT_BLACK);
    tft.setTextSize(2); // Set text size
    tft.println(output);
  
}

///////////////////////////////////////////////////////////////////////////////////////////
// BREW PROFILES | UNDER DEVELOPMENT //////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void BrewSwitch(){
 // TODO: Read when the brew switch has been pressed; view to trigger pressure control (PumpPID) routines onoce deployed for pressure profiling.

Brewing = digitalRead(BREW_SW_PIN) == LOW; //Check if brew switch is on

  if (Brewing) {
    // Run the PID routine here
    //
  } else {
    // Alternate code
  }


}


///////////////////////////////////////////////////////////////////////////////////////////
// Physical Inuts (BUTTONS, not in use) ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void CheckButtons(){
 
  buttonA.loop();    // Check button press
  buttonB.loop();    // Check button press

}

// Set to increase & Decrease the setpoint | Yet to set up the rotary encoder button press (i.e. enter menu)

void click(Button2& btn) {
    if (btn == buttonA) {
      setpoint++;
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(10,40);
    tft.println(setpoint);
    Serial.print("UP");
    store_setpoint();
    // delay(1000);
    tft.fillScreen(TFT_BLACK);
    
    } else if (btn == buttonB) {
     setpoint--;
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(10,40);
    tft.println(setpoint);
    Serial.print("DOWN");
    store_setpoint();
    // delay(1000);
    tft.fillScreen(TFT_BLACK);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// SPIFFS Routines. /////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

 void store_setpoint() {

  // Open file for writing
  File file = SPIFFS.open("/setpoint.txt", "w");
  if (!file) {
    Serial.println("Error opening file for writing!");
    return;
  }
  // Write setpoint value to file
  file.println(setpoint);
  file.println(Kp);
  file.println(Ki);
  file.println(Kd);
  file.println(pumpsetpoint);
  file.println(pumpKp);
  file.println(pumpKi);
  file.println(pumpKd);

  // Close file
  file.close();
 }

void read_setpoint() {
  
  // Recover the past value for setpoint
  // Set up SPIFFS
  if (!SPIFFS.begin(true)) { // Mount filesystem
    Serial.println("Error mounting SPIFFS filesystem!");
    return;
  }
  // Open file for reading
  File file = SPIFFS.open("/setpoint.txt", "r");
  if (!file) {
    Serial.println("Error opening file for reading!");
    return;
  }
    // Read setpoint value, Kp, Ki and Kd from file
  String value = file.readStringUntil('\n');
  if (value.length() > 0) {
    setpoint = value.toInt();
  }
  value = file.readStringUntil('\n');
  if (value.length() > 0) {
    Kp = value.toDouble();
  }
  value = file.readStringUntil('\n');
  if (value.length() > 0) {
    Ki = value.toDouble();
  }
  value = file.readStringUntil('\n');
  if (value.length() > 0) {
    Kd = value.toDouble();
  }
    if (value.length() > 0) {
    pumpsetpoint = value.toInt();
  }
  value = file.readStringUntil('\n');
  if (value.length() > 0) {
    pumpKp = value.toDouble();
  }
  value = file.readStringUntil('\n');
  if (value.length() > 0) {
    pumpKi = value.toDouble();
  }
  value = file.readStringUntil('\n');
  if (value.length() > 0) {
    pumpKd = value.toDouble();
  }
  // Close file
  file.close();
}

///////////////////////////////////////////////////////////////////////////////////////////
// Logger | UNDER DEVELOPMENT /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void appendToSystemLog(String logMessage) {
  // Open the file in append mode
  File file = SPIFFS.open("/Systemlog", FILE_APPEND);
  // Write the message to the file followed by a newline character
  file.println(logMessage);
  // Close the file
  file.close();
}


///////////////////////////////////////////////////////////////////////////////////////////
// TFT Screen Draw ////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void DrawHomeScreen(){
  // Draw UI on main TFT
  if (drawit) // Check for temperature tuning interface bool
    {
        TempGraph(input, setpoint); // Print temperature on display
        tft.setCursor(10,10); // Set first line position
        tft.printf("%.1f",input);

    } else {

        tft.setTextColor(TFT_WHITE,TFT_BLACK); //Set text colour
        tft.setTextSize(3); // Set text size
        // Print temperature on display
        tft.setCursor(210,73); // Set first line position
        tft.printf("%.1f",input);
        // Print target setpoint:
        tft.setCursor(48,73); // Set first line position
        tft.printf("%.1f",setpoint);
        IPAddress apIP = WiFi.softAPIP();
        // IPAddress ip = WiFi.localIP();
        tft.setTextColor(TFT_WHITE,TFT_BLACK);
        tft.setCursor(35,222); // Set first line position
        tft.setTextSize(1); // Set text size
        tft.println(apIP);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Bitmap background write ////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

// Look to replace this with tft_eSPI drawn interface; not bitmap to reduce memory overhead. 

void bootimageold() {

   // Initialize SPIFFS
  SPIFFS.begin();

  // Open the file for reading
  File file = SPIFFS.open("/bootimage.bmp", "r");
  
  // Check if the file is open
  if (!file) {
    Serial.println("Error opening file");
    return;
  }

  // Read the file into a buffer
  size_t fileSize = file.size();
  uint8_t *buffer = (uint8_t *)malloc(fileSize);
  file.read(buffer, fileSize);

  // Draw the image on the TFT display
  //tft.drawBitmap(0, 0, buffer, 320, 240, TFT_BLACK);
  tft.pushImage(0, 0, 320, 240, buffer);

  // Close the file and free the memory
  file.close();
  free(buffer);

  delay(2000);
 
}

void bootimagenew(){

  int16_t rc = png.openFLASH((uint8_t *)backgroundloading, sizeof(backgroundloading), pngDraw);
  if (rc == PNG_SUCCESS) {
   tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    tft.endWrite();
    // png.close(); // not needed for memory->memory decode
  }

}

void infopagedraw(){

  int16_t rc = png.openFLASH((uint8_t *)infopageb, sizeof(infopageb), pngDraw);
  if (rc == PNG_SUCCESS) {
   tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    tft.endWrite();
    // png.close(); // not needed for memory->memory decode
  }

}

// This next function will be called during decoding of the png file to render each image line to the TFT

void pngDraw(PNGDRAW *pDraw) {

  uint16_t lineBuffer[MAX_IMAGE_WDITH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer); // remember too set colour depth (eg 24bit)

}

///////////////////////////////////////////////////////////////////////////////////////////
// Charting for TFT temperature display ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

// VERY VERY Draft, just to try and visualise what's happening on the PID autotune, needs a lot of work

void TempGraph(double temperature, double setline) {
  
  // Set the maximum and minimum temperature values for the graph
  double maxTemperature = (setpoint + 30);
  double minTemperature = (setpoint - 30);

  // Store the temperature value in the array
  temperatures[currentIndex] = temperature;
  currentIndex++;

  if (currentIndex >= 120) {
    currentIndex = 0;
  }

  // Clear the screen
  tft.fillScreen(TFT_BLACK);

  // Scale the setpoint value to fit within the graph dimensions
  int setpointY = map(setline, minTemperature, maxTemperature, graphHeight, 0);
  int32_t r = 5;

  // Draw a horizontal line across the chart to represent the setpoint
  tft.drawFastHLine(graphX, setpointY, graphWidth, TFT_RED);

  // Draw a pixel for each temperature value in the array
  for (int i = 0; i < 120; i++) {
    // Scale the temperature value to fit within the graph dimensions
    int y = map(temperatures[i], minTemperature, maxTemperature, graphHeight, 0);

    // Calculate the x position for the current temperature value
    int x = map(i, 0, 60, graphX, graphX+graphWidth);

    // Draw a pixel at the current temperature value
    //tft.drawPixel(x, y, TFT_WHITE);
    tft.drawCircle(x, y, r, TFT_WHITE);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
// ## WEB USER INTERFACE ## ///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

// Root webserver page and destinations

void serverwebpages() {

  server.on("/", handle_OnConnect);
  server.on("/setpoint", handle_setpoint);
  server.on("/tuneon", handletuneon);
  server.on("/tuneoff", handletuneoff);
  server.on("/simon", handlesimon);
  server.on("/simoff", handlesimoff);
  server.on("/draw", handledraw);
  server.on("/drawstop", handledrawoff);

}

void handle_OnConnect() {
  // Read current setpoint value
  double currentSetpoint = setpoint;
  double currentKp = Kp;
  double currentKi = Ki;
  double currentKd = Kd;

  // Build the HTML page 
  String html = "<html><body><h1>ESP Coffee PID</h1>";
  html += "<p>Current PID target temperature: " + String(currentSetpoint) + "</p>";
  html += "<p>Current Kp: " + String(currentKp) + "</p>";
  html += "<p>Current Ki: " + String(currentKi) + "</p>";
  html += "<p>Current Kd: " + String(currentKd) + "</p>";

  html += "<form action='/setpoint' method='post'>";
  html += "  <label for='setpoint'>Enter new PID target:</label><br>";
  html += "  <input type='text' id='setpoint' name='setpoint' value='" + String(currentSetpoint) + "'><br>";
  html += "  <label for='Kp'>Enter new Kp:</label><br>";
  html += "  <input type='text' id='Kp' name='Kp' value='" + String(currentKp) + "'><br>";
  html += "  <label for='Ki'>Enter new Ki:</label><br>";
  html += "  <input type='text' id='Ki' name='Ki' value='" + String(currentKi) + "'><br>";
  html += "  <label for='Kd'>Enter new Kd:</label><br>";
  html += "  <input type='text' id='Kd' name='Kd' value='" + String(currentKd) + "'><br>";
  html += "  <input type='submit' value='Submit'>";
  html += "  <br><br>";
  html += "<a href='/tuneon' target='_blank'>Autotune PID ON</a>";
  html += "<a href='/tuneoff' target='_blank'>Autotune PID OFF</a>";
  html += "  <br><br>";
  html += "<a href='/simon' target='_blank'>Autotune SIMULATION ON</a>";
  html += "<a href='/simoff' target='_blank'>Autotune SIMULATION OFF</a>";
  html += "  </form>";
  
  // Send the HTML page to the client
  server.send(200, "text/html", html);
}

// Functional html pages 

void handle_setpoint() {
 // Parse setpoint and PID gain values from HTTP request
  String setpointValue = server.arg("setpoint");
  String kpValue = server.arg("Kp");
  String kiValue = server.arg("Ki");
  String kdValue = server.arg("Kd");
  double newSetpoint = setpointValue.toDouble();
  double newKp = kpValue.toDouble();
  double newKi = kiValue.toDouble();
  double newKd = kdValue.toDouble();

  // Update setpoint and PID gain variables in PID code
  setpoint = newSetpoint;
  Kp = newKp;
  Ki = newKi;
  Kd = newKd;

  // Update PID controller with new gains
  myPID.SetTunings(Kp, Ki, Kd);

  // Build the HTML page
  String html = "<html><body><h1>ESP Coffee PID</h1>";
  html += "<p>PID target and gains updated to:</p>";
  html += "<ul>";
  html += "  <li>Setpoint: " + String(newSetpoint) + "</li>";
  html += "  <li>Kp: " + String(newKp) + "</li>";
  html += "  <li>Ki: " + String(newKi) + "</li>";
  html += "  <li>Kd: " + String(newKd) + "</li>";
  html += "</body></html>";

  // Send the HTML page to the client
  server.send(200, "text/html", html);

  store_setpoint();
}

void handledraw() { // Autotune temp graph on TFT ON
  drawit = true;
  server.send(200, "text/plain", "Draw On.");
}

void handledrawoff() { // Autotune temp graph on TFT OFF

  drawit = false;
  tft.fillScreen(TFT_BLACK);
  infopagedraw(); // draw background image
  server.send(200, "text/plain", "Draw Off.");
}

void handletuneon() { // Boiler PID Autotune ON
  tuning = true;
  server.send(200, "text/plain", "Tune On.");
}

void handletuneoff() { // Boiler PID Autotune ON
  tuning = false;
  server.send(200, "text/plain", "Tune Off.");
}

void handlesimon() { // Boiler PID Autotune simulation ON
  useSimulation = true;
  server.send(200, "text/plain", "Sim On.");
}

void handlesimoff() { // Boiler PID Autotune simulation OFF
  useSimulation = false;
  server.send(200, "text/plain", "Sim Off.");
}

