// Date: 16 December 2022
// Author: Andrew Sweetman
// Basic thermoscouple read and write to TFT display with WiFi Manager and web server

// Libraries
#include <NTPClient.h>
#include <max6675.h> // Thermocouple library
#include <PID_v1.h> // PID library
#include <SPI.h> 
#include <time.h>
#include "SPIFFS.h" // Add SPIFFS library
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <WiFiManager.h> // WiFi Manager library
#include <WebServer.h> // ESP32 web server library
#include <WiFiUdp.h> // UDP library
#include <wiFi.h> // WiFi library
#include <Button2.h> // Button handler

WiFiManager wifiManager; // WiFi Manager object

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Variables to save date and time
String currentTime;

// Define ESP32 Pins
int soPin = 12;// MISO=Serial Out
int csPin = 15;// CS = chip select CS pin
int sckPin = 2;// SCK = Serial Clock pin
int siPin = 13; // MOSI 
int relayPin = 17; //Set boilder SSR control pin here

#define BUTTON_A_PIN  35
#define BUTTON_B_PIN  0

Button2 buttonA = Button2(BUTTON_A_PIN);
Button2 buttonB = Button2(BUTTON_B_PIN);

MAX6675 thermocouple(sckPin, csPin, soPin);// create instance object of MAX6675

// PID gains
double Kp = 1, Ki = 0.1, Kd = 0.01;

// Input, output, setpoint
double input = 0, output = 0, setpoint = 50;
double oldSetpoint = setpoint; // for screen clearing


// Create PID object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

WebServer server(80); // Create a web server on port 80

TFT_eSPI tft = TFT_eSPI(); // Define the tft object

// Define new colours
#define TFT_GREY 0x5AEB // New colour

////////////////////////////////////////////////////////////////////
// setup function

void setup(void) {

  Serial.begin(115200);// initialize serial monitor

  // Recover the past value for setpoint
  read_setpoint();

  // Close file
  file.close();

  // button handler
  buttonA.setClickHandler(click);
  buttonB.setClickHandler(click);

  // Initialize TFT display 
  tft.init(); // Initialize display
  tft.setRotation(1); // Set rotation to landscape
  tft.fillScreen(TFT_BLACK); // Clear screen
  tft.setTextSize(3);
  tft.setCursor(10,40);
  tft.println("Setup Wifi");
  tft.println("ESP32BrewPID");
  
  wifiManager.autoConnect("ESP32BrewPID"); // Connect to WiFi

  tft.fillScreen(TFT_BLACK); // Clear screen
  tft.setCursor(10,40);
  tft.println("Welcome to");
  tft.println("ESP32BrewPID");
  delay(3000);
  tft.fillScreen(TFT_BLACK); // Clear screen

  // // Initialize a NTPClient to get time
  timeClient.begin();
  // // Set offset time in seconds to adjust for your timezone, for example:
  // // GMT +1 = 3600
  // // GMT +8 = 28800
  // // GMT -1 = -3600
  // // GMT 0 = 0
  timeClient.setTimeOffset(28800);

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);

  server.on("/", handle_OnConnect);
  server.on("/setpoint", handle_setpoint);

  server.begin(); // Start web server

}

////////////////////////////////////////////////////////////////////
// Main loop

void loop() {

// Handle client requests
  server.handleClient();

  // Update the time every minute
  if (timeClient.update()) {
    String currentTime = timeClient.getFormattedTime();
  }
  
  tft.setTextColor(TFT_GREEN,TFT_BLACK); //Set text colour
  tft.setTextSize(2); //Set text size
  tft.setCursor(0,5); //Set first line position

  input = thermocouple.readCelsius();   // Read temperature from thermocouple
  
 // Print temperature on display
  tft.print("Temperature: ");
  tft.println(input);

  // Print target setpoint:
  tft.print("PID Target: ");
  tft.println(setpoint);

  // Calculate PID output
  myPID.Compute();
  
  // Use PID output to control relay
  if (output > 0) {
    digitalWrite(relayPin, HIGH); // Turn the relay on
    tft.println("Heater ON");
  } else {
    digitalWrite(relayPin, LOW); // Turn the relay off
    tft.println("Heater OFF");
  }
  
 // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  tft.println(" ");
  tft.println(ip);

  // tft.println(currentTime); 
 
  // Check button press
  buttonA.loop();
  buttonB.loop();

 oldSetpoint = setpoint;

  Serial.print("loopedyloop");

//  while(1) yield(); // We must yield() to stop a watchdog timeout.
}

////////////////////////////////////////////////////////////////////

void click(Button2& btn) {
    if (btn == buttonA) {
      setpoint++;
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(10,40);
    tft.println(setpoint);
    Serial.print("UP");
    store_setpoint();
    delay(1000);
    tft.fillScreen(TFT_BLACK);
    
    } else if (btn == buttonB) {
     setpoint--;
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(10,40);
    tft.println(setpoint);
    Serial.print("DOWN");
    store_setpoint();
    delay(1000);
    tft.fillScreen(TFT_BLACK);
    }
}

////////////////////////////////////////////////////////////////////

void handle_OnConnect() {
  // Read current setpoint value
  double currentSetpoint = setpoint;

  // Build the HTML page
  String html = "<html><body><h1>ESP32BrewPID</h1>";
  html += "<p>Current setpoint: " + String(currentSetpoint) + "</p>";
  html += "<form action='/setpoint' method='post'>";
  html += "  <label for='setpoint'>Setpoint:</label><br>";
  html += "  <input type='text' id='setpoint' name='setpoint' value='" + String(currentSetpoint) + "'><br>";
  html += "  <input type='submit' value='Submit'>";
  html += "</form>";
  html += "</body></html>";

  // Send the HTML page to the client
  server.send(200, "text/html", html);
}

////////////////////////////////////////////////////////////////////

void handle_setpoint() {
  // Parse setpoint value from HTTP request
  String setpointValue = server.arg("setpoint");
  double newSetpoint = setpointValue.toDouble();

  // Update setpoint variable in PID code
  setpoint = newSetpoint;

  // Build the HTML page
  String html = "<html><body><h1>ESP32BrewPID</h1>";
  html += "<p>Setpoint updated to: " + String(newSetpoint) + "</p>";
  html += "</body></html>";

  // Send the HTML page to the client
  server.send(200, "text/html", html);

  store_setpoint();
}

////////////////////////////////////////////////////////////////////

 void store_setpoint() {

  // Open file for writing
  File file = SPIFFS.open("/setpoint.txt", "w");
  if (!file) {
    Serial.println("Error opening file for writing!");
    return;
  }

  // Write setpoint value to file
  file.println(setpoint);

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

  // Read setpoint value from file
  String value = file.readStringUntil('\n');
  if (value.length() > 0) {
    setpoint = value.toInt();
  }
}