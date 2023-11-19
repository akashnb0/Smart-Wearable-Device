#include <SoftwareSerial.h>

#include <TinyGPS++.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
//--------------------------------------------------------------

//-------------------------------
static const int RXPin = 8, TXPin = 9;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin); // for gps
/////////////////////////////////////////////////////
String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte destination = 0xFF;
byte localAddress = 0xBB;
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

///-------------------------------
String Mymessage = "";
//--------------------------------------------------------------
// Size of the geo fence (in meters)
const float maxDistance = 30;

//--------------------------------------------------------------
//float initialLatitude = 34.014875;
//float initialLongitude = 72.163585;

float latitude, longitude;

char buff[10];
String mylong = ""; // for storing the longittude value
String mylati = ""; // for storing the latitude value

//--------------------------------------------------------------

int msgstatus;

int Sensor1;
int relay = 3;
float distance;

/*****************************************************************************************
 * setup() function
 *****************************************************************************************/
void setup()
{
  //--------------------------------------------------------------
  //Serial.println("Arduino serial initialize");
  Serial.begin(9600);

  //--------------------------------------------------------------
  //Serial.println("NEO6M serial initialize");
    ss.begin(GPSBaud);
  //--------------------------------------------------------------
  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

 Serial.println("LoRa init succeeded.");
}

/*****************************************************************************************
 * loop() function
 *****************************************************************************************/
void loop()
{

  while (ss.available() > 0)
    if  ( gps.encode(ss.read() ) )
    {

  latitude = gps.location.lat(), 6 ;
  Serial.print(latitude);
  Serial.print(longitude);
  longitude = gps.location.lng(), 6 ;
  mylati = dtostrf(latitude, 3, 6, buff);
  mylong = dtostrf(longitude, 3, 6, buff);
//  distance = getDistance(latitude, longitude, initialLatitude, initialLongitude);

    }

  //--------------------------------------------------------------
//  if (millis() - lastSendTime > interval) {
//  //displayInfo();
//
// // Serial.print("Latitude= "); Serial.println(latitude, 6);
//  //Serial.print("Lngitude= "); Serial.println(longitude, 6);
//
//    if(distance > maxDistance) {
//      msgstatus =1;
//    }
//
//    if(distance < maxDistance) {
//      msgstatus =0;
//
//    }

    // Serial.print("Distance: ");
  //Serial.println(distance);
   Mymessage = Mymessage + mylati +"," + mylong;
     sendMessage(Mymessage);
     //Serial.println(Mymessage);
    delay(50);
    Mymessage = "";



      //Serial.println("Sending " + message);
             // timestamp the message

  }

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}
