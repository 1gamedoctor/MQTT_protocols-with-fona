/***************************************************
  Adafruit MQTT Library FONA Example*/

#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

/************BMP**********/
#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;
#define ALTITUDE 1702.0 //base default altitude 

const int dry = 876; //value for a dry sensor
const int wet = 239; //value for a wet sensor

/*************************** FONA Pins ***********************************/

// Default pins for Feather 32u4 FONA
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

/************************* WiFi Access Point *********************************/

  // Optionally configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.
#define FONA_APN       "safaricom"
#define FONA_USERNAME  "saf"
#define FONA_PASSWORD  "data"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "1gamedoctor"
#define AIO_KEY         "******"

/************ Global State (you don't need to change this!) ******************/

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/

// Setup a feed called  for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temperaturepub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish pressurepub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
Adafruit_MQTT_Publish altitudepub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/altitude");
Adafruit_MQTT_Publish moisturepub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture");

// Setup a feed called 'onoff' for subscribing to changes.
//Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;
#define MAXTXFAILURES 3

void setup() {
  while (!Serial);

  // Watchdog is optional!
  //Watchdog.enable(8000);
  
  Serial.begin(115200);
    if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
  Serial.println(F("Adafruit FONA MQTT demo"));

  //mqtt.subscribe(&onoffbutton);

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
  
  // Initialise the FONA module
  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
    Serial.println("Retrying FONA");
  }

  Serial.println(F("Connected to Cellular!"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
}

uint32_t x=0;

void loop() {
  //fonaSS.println("AT+CSCLK=2"); //Put fona into sleep mode after 5 seconds of inactivity
  char status;
  double T,P,p0,a;   //variables for pressure, temperature, altitude
  int sensorVal = analogRead(A0);
  float percentageMoisture = map(sensorVal, wet, dry, 100,0); //converts the moisture values to percentages
  status = pressure.startTemperature();
  delay(500);
  status = pressure.getTemperature(T);
  Serial.print("temperature: ");
  Serial.print(T,2);// temp in Celcius
  status = pressure.startPressure(3);
  status = pressure.getPressure(P,T);
   p0 = pressure.sealevel(P,ALTITUDE); // we're at 1702 for strathmore library ground floor
          Serial.print(p0,2); //relative sea level pressure
  Serial.print("absolute pressure: ");
          Serial.print(P,2);//pressure in millibars
  a = pressure.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);//altitude in meter
  // Make sure to reset watchdog every loop iteration!
  Watchdog.reset();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Watchdog.reset();
  // Now we can publish stuff!
  Serial.print(F("\nSending values to Adafruit IO "));
  Serial.print("...");
  if (! temperaturepub.publish(T)&!pressurepub.publish(P)&!altitudepub.publish(a)&!moisturepub.publish(percentageMoisture)) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
    delay(5000);
  }

  Watchdog.reset();  
  // this is our 'wait for incoming subscription packets' busy subloop
  /*Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);
    }
  }*/

  // ping the server to keep the mqtt connection alive, only needed if we're not publishing
  //if(! mqtt.ping()) {
  //  Serial.println(F("MQTT Ping failed."));
  //}

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
