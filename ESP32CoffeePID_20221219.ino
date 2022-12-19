// Basic PID control for a single boiler esprssp machine, with web interface

// Libraries
#include <Button2.h> // Button handler
#include <max6675.h> // Thermocouple library
#include <PID_v1.h> // PID library
#include <PNGdec.h>
#include <SPI.h> 
#include "SPIFFS.h" // Add SPIFFS library
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
// #include <WiFiManager.h> // WiFi Manager library
#include <WebServer.h> // ESP32 web server library
#include <WiFiUdp.h> // UDP library
#include <WiFi.h> // WiFi library
#include <WiFiClient.h>
#include <WiFiAP.h>

const char* ssid = "ESP Coffee PID"; // Set the name of the AP
const char* password = "";  // Set the password for the AP (leave blank for open AP)

IPAddress local_IP(192, 168, 1, 1);  // Set the host IP for the AP
IPAddress gateway(192, 168, 1, 1);   // Set the gateway IP
IPAddress subnet(255, 255, 255, 0);  // Set the subnet mask

// image arrays
#include "backgroundloading.h" // Startup boot screen image
#include "infopageb.h" //main page backdrop

#define MAX_IMAGE_WDITH 320 // Adjust for your images

int16_t xpos = 0;
int16_t ypos = 0;

// Define ESP32 Pins
int soPin = 12;// MISO=Serial Out
int csPin = 15;// CS = chip select CS pin
int sckPin = 14;// SCK = Serial Clock pin
int siPin = 13; // MOSI 
int relayPin = 32; //Set boilder SSR control pin here

// Buttons (if used)
#define BUTTON_A_PIN  35 // set pins required
#define BUTTON_B_PIN  0 // set pins as required

// PID gains
double Kp = 1, Ki = 0.1, Kd = 0.01;

// Input, output, setpoint
double input = 0, output = 0, setpoint = 93;

// object instances
PNG png; // PNG decoder inatance
// WiFiManager wifiManager; // WiFi Manager object
MAX6675 thermocouple(sckPin, csPin, soPin);// create instance object of MAX6675
TFT_eSPI tft = TFT_eSPI(); // Define the tft object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // Create PID object

Button2 buttonA = Button2(BUTTON_A_PIN);
Button2 buttonB = Button2(BUTTON_B_PIN);

WebServer server(80); // Create a web server on port 

////////////////////////////////////////////////////////////////////

void setup(void) {

  Serial.begin(9600);// initialize serial monitor

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
  
  myPID.SetMode(AUTOMATIC); // Initialize PID controller

  server.on("/", handle_OnConnect);
  server.on("/setpoint", handle_setpoint);

  server.begin(); // Start web server

  pinMode(relayPin, OUTPUT);

  // button handler
  buttonA.setClickHandler(click);
  buttonB.setClickHandler(click);

}

////////////////////////////////////////////////////////////////////

void loop() {

  input = thermocouple.readCelsius();   // Read temperature from thermocouple
  delay(500);

  server.handleClient(); // Handle client requests
  
  tft.setTextColor(TFT_WHITE,TFT_BLACK); //Set text colour
  tft.setTextSize(3); //Set text size
  
 // Print temperature on display
  tft.setCursor(210,73); //Set first line position
  tft.printf("%.1f",input);

  // Print target setpoint:
  tft.setCursor(48,73); //Set first line position
  tft.printf("%.1f",setpoint);

  // Calculate PID output
  myPID.Compute();
  
  // Use PID output to control relay
  if (output > 0) {
    digitalWrite(relayPin, HIGH); // Turn the relay on
    tft.setCursor(118,175); //Set first line position
    tft.setTextColor(TFT_GREEN,TFT_BLACK);
    tft.setTextSize(2); //Set text size
    tft.println("HEATING");
  } else {
    digitalWrite(relayPin, LOW); // Turn the relay off
    tft.setCursor(127,175); //Set first line position
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setTextSize(2); //Set text size
    tft.println(" OFF      ");
  }
  
 // print your board's IP address:

  IPAddress apIP = WiFi.softAPIP();
  // IPAddress ip = WiFi.localIP();
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(35,222); //Set first line position
  tft.setTextSize(1); //Set text size
  tft.println(apIP);
 
  // Check button press
  buttonA.loop();
  buttonB.loop();

  Serial.print("Loop");

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
// Web interface //

void handle_OnConnect() {
  // Read current setpoint value
  double currentSetpoint = setpoint;

  // Build the HTML page
  String html = "<html><body><h1>ESP Coffee PID</h1>";
  html += "<p>Current PID target temperature: " + String(currentSetpoint) + "</p>";
  html += "<form action='/setpoint' method='post'>";
  html += "  <label for='setpoint'>Enter new PID target:</label><br>";
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
  String html = "<html><body><h1>ESP Coffee PID</h1>";
  html += "<p>PID target updated to: " + String(newSetpoint) + "</p>";
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

  // Close file
  file.close();


}

////////////////////////////////////////////////////////////////////
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

