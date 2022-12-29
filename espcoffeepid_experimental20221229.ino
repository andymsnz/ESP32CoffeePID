// Basic PID control for a single boiler esprssp machine, with web interface

// Libraries
#include <Button2.h> // Button handler
#include <Encoder.h>
#include <FastLED.h>
#include <max6675.h> // Thermocouple library
#include <PID_v1.h> // PID library
#include <PID_AutoTune_v0.h> // PID autotuning library
#include <PNGdec.h> // PNG image decode
#include <SPI.h> //SPI library
#include "SPIFFS.h" // Add SPIFFS library
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <WebServer.h> // ESP32 web server library
#include <WiFiUdp.h> // UDP library
#include <WiFi.h> // WiFi library
#include <WiFiClient.h>
#include <WiFiAP.h>
// #include <wire.h>


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


// Define ESP32 GPIO Pins (Future WIP, move to pins.h header)

//Brew Switch (not deployed)
// #define BREW_SW_PIN 22 
// Buttons (if used)
#define BUTTON_A_PIN  35 // set pins as required
#define BUTTON_B_PIN  34 // set pins as required
//Rotary encoder
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 4
#define ENCODER_SW_PIN 27
//I2C Bus (not deployed)
// #define I2C_SDA_PIN 3
// #define I2C_SCL_PIN 1
//MAX6675 Thermocouple
#define TC_SO_PIN 12 // MISO=Serial Out
#define TC_CS_PIN 15 // CS = chip select CS pin
#define TC_SCK_PIN 14 // SCK = Serial Clock pin
#define TC_SI_PIN 13 // MOSI 
//Neopixel (not deployed)
#define NEOPIX_PIN 21
#define NUM_LEDS 4 // How may LEDs on the PIN
//Pump AC Dimmer (not deployed)
// #define PUMP_ZC_PIN 26
// #define PUMP_PWM_PIN 25
//Pressure Sensor (not deployed)
// #define PRESSURE_PIN 33
//Solid state relay (SSR)
#define RELAY_PIN 32 //Boiler SSR Control pin (HIGH 3v3)
// SPI SCREEN (SET IN TFT_eSPI Listed here for reference only) || ST7789V 2.0" RGB TFT 320x240 screen specified in this build
// SCL | GPIO 18
// SDA | GPIO 23
// RST | GPIO 16
// DC | GPIO 17
// CS | GPIO 5


// PID Variables

// PID gains (keep a history until it's tuned)
double Kp = 2.78, Ki = 0.05, Kd = 0; // (result of earlier autotune)
double input = 0, output = 0, setpoint = 93; // Input, output, setpoint
double controltype = 0; //Set 0 for PI control and 1 for PID control autotuning parameters

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

// Object instances

Button2 buttonA = Button2(BUTTON_A_PIN); // physical button at pre-defined pin
Button2 buttonB = Button2(BUTTON_B_PIN); // physical button at pre-defined pin
Encoder encoder(2, 4); // 2 and 4 are the pins that the rotary encoder is connected to
MAX6675 thermocouple(TC_SCK_PIN, TC_CS_PIN, TC_SO_PIN);// create instance object of MAX6675
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // Create PID object
PID_ATune aTune(&input, &output);
PNG png; // PNG decoder inatance
TFT_eSPI tft = TFT_eSPI(); // Define the tft object
WebServer server(80); // Create a web server on port
CRGB leds[NUM_LEDS]; // Declare an array to hold the LED colors

unsigned long startTime; // A variable to store the start time of the system


////////////////////////////////////////////////////////////////////
///////////////////////////. SETUP  ////////////////////////////////
////////////////////////////////////////////////////////////////////

void setup(void) {

  Serial.begin(9600);// initialize serial monitor

  FastLED.addLeds<WS2812, NEOPIX_PIN, GRB>(leds, NUM_LEDS); // Initialize the FastLED object

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

 //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
    
  myPID.SetMode(AUTOMATIC); // Initialize PID controller

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

  WiFi.mode(WIFI_AP);  // Set the ESP32 to AP mode
  WiFi.softAPConfig(local_IP, gateway, subnet);  // Configure the AP with the specified host IP, gateway, and subnet mask
  WiFi.softAP(ssid, password);  // Start the AP with the specified name and password

  tft.fillScreen(TFT_BLACK); // Clear screen

  infopagedraw(); // draw background image
  read_setpoint();// Recover the past value for setpoint
  serverwebpages(); //setup webserver browser locations
  server.begin(); // Start web server

  // button handler
  buttonA.setClickHandler(click);
  buttonB.setClickHandler(click);

}


////////////////////////////////////////////////////////////////////
///////////////////////////  MAIN  /////////////////////////////////
////////////////////////////////////////////////////////////////////


void loop() {

  input = thermocouple.readCelsius();   // Read temperature from thermocouple
  delay(500);

  // PID and Autotune routine
  PIDcycle();

  // Read the rotary encoder position
  int position = encoder.read();
  Serial.println(position);
  
  // Main disolay draw
  if (drawit)
  {
  TempGraph(input, setpoint); // Print temperature on display
    tft.setCursor(10,10); //Set first line position
    tft.printf("%.1f",input);

  } else {

  tft.setTextColor(TFT_WHITE,TFT_BLACK); //Set text colour
  tft.setTextSize(3); //Set text size
 // Print temperature on display
  tft.setCursor(210,73); //Set first line position
  tft.printf("%.1f",input);
  // Print target setpoint:
  tft.setCursor(48,73); //Set first line position
  tft.printf("%.1f",setpoint);
  IPAddress apIP = WiFi.softAPIP();
  // IPAddress ip = WiFi.localIP();
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(35,222); //Set first line position
  tft.setTextSize(1); //Set text size
  tft.println(apIP);
}
 
  myPID.Compute();  // Calculate PID output
  RelayDrive();  // Drive the relay
 
  // Check button press
  buttonA.loop();
  buttonB.loop();

  TankLights();

  server.handleClient(); // Handle client requests
  Serial.print("Main Loop Cycle");

}

// NEW LOOPS ////////////////////////////////////////////

void TankLights()
{

  fill_solid(leds, NUM_LEDS, CRGB::Red);  // Set all the LEDs to red
  FastLED.show(); // Update the LEDs

}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the Input
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
  else //This is optimized for Relay output.
  { 
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(output > now - windowStartTime) digitalWrite(RELAY_PIN,HIGH);
  else digitalWrite(RELAY_PIN,LOW);  
}

  // Write the output status variable to the TFT (update later to on/off in loop above)
    tft.setCursor(118,175); //Set first line position
    tft.setTextColor(TFT_GREEN,TFT_BLACK);
    tft.setTextSize(2); //Set text size
    tft.println(output);
  
}


///////////// OLD LOOPS ///////////////

// Button handlers
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

////////////////////////////////////////////////////////////////////
// Root webserver page

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

  // Build the HTML page (adding autotune button)
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

// Functional pages
///////////////////////////////////////////////////////////////////

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

// Handle requests to pid_autotune webpage
void handledraw() {
  drawit = true;
  server.send(200, "text/plain", "Draw On.");
}

// Handle requests to pid_autotune webpage
void handledrawoff() {

  drawit = false;
  tft.fillScreen(TFT_BLACK);
  infopagedraw(); // draw background image
  server.send(200, "text/plain", "Draw Off.");
}

// Handle requests to pid_autotune webpage
void handletuneon() {
  tuning = true;
  server.send(200, "text/plain", "Tune On.");
}

// Handle requests to pid_autotune webpage
void handletuneoff() {
  tuning = false;
  server.send(200, "text/plain", "Tune Off.");
}

// Handle requests to pid_autotune webpage
void handlesimon() {
  useSimulation = true;
  server.send(200, "text/plain", "Sim On.");
}

// Handle requests to pid_autotune webpage
void handlesimoff() {
  useSimulation = false;
  server.send(200, "text/plain", "Sim Off.");
}




// SPIFFS Routines

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

  // Close file
  file.close();
 }

////////////////////////////////////////////////////////////////////

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
  // Close file
  file.close();
}


// Bitmap background write //

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

// This next function will be called during decoding of the png file to render each image line to the TFT. 

void pngDraw(PNGDRAW *pDraw) {

  uint16_t lineBuffer[MAX_IMAGE_WDITH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer); // remember too set colour depth (eg 24bit)

}


void RelayDrive(){

  // turn the output pin on/off based on pid output 

  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)    //time to shift the Relay Window
    { 
    windowStartTime += WindowSize;
    }
  if(output > now - windowStartTime) digitalWrite(RELAY_PIN,HIGH);
  else digitalWrite(RELAY_PIN,LOW);

  double controlfraction = ((output / WindowSize) * 100);

    tft.setCursor(118,175); //Set first line position
    tft.setTextColor(TFT_GREEN,TFT_BLACK);
    tft.setTextSize(2); //Set text size
    // tft.println(output);
    tft.println(String(controlfraction) + "%");

}

// Charting for TFT temperature display
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









