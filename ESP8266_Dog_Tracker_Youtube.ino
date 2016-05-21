
#include "Adafruit_MQTT.h"                                  // Adafruit MQTT library
#include "Adafruit_MQTT_Client.h"                           // Adafruit MQTT library

#include "ESP8266WiFi.h"                                    // ESP8266 library     
#include <Adafruit_ssd1306syp.h>                            // Adafruit Oled library for display

#include <TinyGPS++.h>                                      // Tiny GPS Plus Library
#include <SoftwareSerial.h>                                 // Software Serial Library so we can use Pins for communication with the GPS module

#define SDA_PIN 4                                           // uses GPIO pins 4(SDA) and 5(SCL) of the ESP8266 Adafruit Feather
#define SCL_PIN 5                                           // also known as pins D1(SCL) and D2(SDA) of the NodeMCU ESP-12
Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);               // Set OLED display pins

static const int RXPin = 12, TXPin = 13;                    // Ublox 6m GPS module to pins 12 and 13
static const uint32_t GPSBaud = 9600;                       // Ublox GPS default Baud Rate is 9600

TinyGPSPlus gps;                                            // Create an Instance of the TinyGPS++ object called gps
SoftwareSerial ss(RXPin, TXPin);                            // The serial connection to the GPS device

const double HOME_LAT = 32.428348;                          // Enter Your Latitude and Longitude here
const double HOME_LNG = -81.738143;                         // to track how far away the "Dog" is away from Home 

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "warroom2"                          // Enter Your router SSID
#define WLAN_PASS       "1234567890"                        // Enter Your router Password

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                                // use 8883 for SSL
#define AIO_USERNAME    "mkconner"                          // Enter Your Adafruit IO Username
#define AIO_KEY         "ae46b78dec7c48c3af2e7a0784f9781e"  // Enter Your Adafruit IO Key

/************ Global State (you don't need to change this!) ******************/

WiFiClient client;                                          // Create an ESP8266 WiFiClient class to connect to the MQTT server.

const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;           // Store the MQTT server, username, and password in flash memory.
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

// Setup a feed called 'gpslat' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>   // This feed is not needed, only setup if you want to see it
const char gpslat_FEED[] PROGMEM = AIO_USERNAME "/feeds/gpslat";            
Adafruit_MQTT_Publish gpslat = Adafruit_MQTT_Publish(&mqtt, gpslat_FEED);

// Setup a feed called 'gpslng' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>   // This feed is not needed, only setup if you want to see it
const char gpslng_FEED[] PROGMEM = AIO_USERNAME "/feeds/gpslng";
Adafruit_MQTT_Publish gpslng = Adafruit_MQTT_Publish(&mqtt, gpslng_FEED);

// Setup a feed called 'gps' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char gps_FEED[] PROGMEM = AIO_USERNAME "/feeds/gpslatlng/csv";          // CSV = commas seperated values
Adafruit_MQTT_Publish gpslatlng = Adafruit_MQTT_Publish(&mqtt, gps_FEED);

/****************************************************/

void setup() 
{
  Serial.begin(115200);                                 // Setup Serial Comm for Serial Monitor @ 115200 baud
  WiFi.mode(WIFI_STA);                                  // Setup ESP8266 as a wifi station
  WiFi.disconnect();                                    // Disconnect if needed
  delay(100);                                           // short delay
  
  display.initialize();  // init OLED display
  display.clear();                                      // Clear OLED display
  display.setTextSize(1);                               // Set OLED text size to small
  display.setTextColor(WHITE);                          // Set OLED color to White
  display.setCursor(0,0);                               // Set cursor to 0,0
  display.println("  Adafruit IO GPS");  
  display.println("      Tracker"); 
  display.print("---------------------");
  display.update();                                     // Update display
  delay(1000);                                          // Pause X seconds  
    
  ss.begin(GPSBaud);                                    // Set Software Serial Comm Speed to 9600     
   
  display.print("Connecting to WiFi");
  display.update();

  WiFi.begin(WLAN_SSID, WLAN_PASS);                     // Start a WiFi connection and enter SSID and Password
      while (WiFi.status() != WL_CONNECTED) 
         {                                              // While waiting on wifi connection, display "..."
           delay(500);
           display.print(".");
           display.update();
         } 
           display.println("Connected");
           display.update();             
}                                                       // End Setup

void loop() {

  smartDelay(500);                                      // Update GPS data TinyGPS needs to be fed often
  MQTT_connect();                                       // Run Procedure to connect to Adafruit IO MQTT  

  float Distance_To_Home;                               // variable to store Distance to Home  
  float GPSlat = (gps.location.lat());                  // variable to store latitude
  float GPSlng = (gps.location.lng());                  // variable to store longitude
  float GPSalt = (gps.altitude.feet());                 // variable to store altitude  
  Distance_To_Home = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),HOME_LAT, HOME_LNG);  //Query Tiny GPS to Calculate Distance to Home
    
  display.clear();    
  display.setCursor(0,0);  
  display.println(F("  GPS Tracking"));
  display.print("---------------------");
  display.update();      

  display.print("GPS Lat: ");
  display.println(gps.location.lat(), 6);               // Display latitude to 6 decimal points
  display.print("GPS Lon: ");
  display.println(gps.location.lng(), 6);               // Display longitude to 6 decimal points
  display.print("Distance: ");
  display.println(Distance_To_Home);                    // Distance to Home measured in Meters    
  display.update();     

  // ********************** Combine Data to send to Adafruit IO *********************************
  // Here we need to combine Speed, Latitude, Longitude, Altitude into a string variable buffer to send to Adafruit    
                                                            
            char gpsbuffer[30];                         // Combine Latitude, Longitude, Altitude into a buffer of size X
            char *p = gpsbuffer;                        // Create a buffer to store GPS information to upload to Adafruit IO                       

            dtostrf(Distance_To_Home, 3, 4, p);         // Convert Distance to Home to a String Variable and add it to the buffer
            p += strlen(p);
            p[0] = ','; p++;                      
            
            dtostrf(GPSlat, 3, 6, p);                   // Convert GPSlat(latitude) to a String variable and add it to the buffer
            p += strlen(p);
            p[0] = ','; p++;
                                                            
            dtostrf(GPSlng, 3, 6, p);                   // Convert GPSlng(longitude) to a String variable and add it to the buffer
            p += strlen(p);
            p[0] = ','; p++;  
                                                            
            dtostrf(GPSalt, 2, 1, p);                   // Convert GPSalt(altimeter) to a String variable and add it to the buffer
            p += strlen(p);
                                                                        
            p[0] = 0;                                   // null terminate, end of buffer array

            if ((GPSlng != 0) && (GPSlat != 0))         // If GPS longitude or latitude do not equal zero then Publish
              {
              display.println("Sending GPS Data ");     
              display.update(); 
              gpslatlng.publish(gpsbuffer);             // publish Combined Data to Adafruit IO
              Serial.println(gpsbuffer);  
              }
            
            gpslng.publish(GPSlng,6);                   // Publish the GPS longitude to Adafruit IO                 
            
            if (! gpslat.publish(GPSlat,6))             // Publish the GPS latitude to Adafruit IO
               {
                 display.println(F("Failed"));          // If it failed to publish, print Failed
               } else 
                  {
                   //display.println(gpsbuffer);
                   display.println(F("Data Sent!"));                   
                   }  
        display.update();
        delay(1000);         
    
  if (millis() > 5000 && gps.charsProcessed() < 10)
    display.println(F("No GPS data received: check wiring"));
  
  // Wait a bit before scanning again
  display.print("Pausing...");
  display.update();
  smartDelay(500);                                      // Feed TinyGPS constantly
  delay(1000);
}


// **************** Smart delay - used to feed TinyGPS ****************

static void smartDelay(unsigned long ms)                 
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


// **************** MQTT Connect - connects with Adafruit IO *****************
void MQTT_connect() {
  
  int8_t ret;
  if (mqtt.connected()) { return; }                     // Stop and return to Main Loop if already connected to Adafruit IO
  display.print("Connecting to MQTT... ");
  display.update();

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {                 // Connect to Adafruit, Adafruit will return 0 if connected
       display.println(mqtt.connectErrorString(ret));   // Display Adafruits response
       display.println("Retrying MQTT...");
       mqtt.disconnect();
       display.update();
       delay(5000);                                     // wait X seconds
       retries--;
       if (retries == 0) {                              // basically die and wait for WatchDogTimer to reset me                                                          
         while (1);         
       }
  }
  display.println("MQTT Connected!");
  display.update();
  delay(1000);
}
