#include <Arduino.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <Adafruit_INA219.h>
#include <SparkFun_MAG3110.h>

#define csPin 10
#define resetPin 9
#define irqPin 2

#define ledOne 8
#define ledTwo 3
#define button A2    //the on-board push switch, for debugging mostly

#define relayOne 4
#define relayTwo 5
#define relayThree 6
#define relayFour 7

MAG3110 mag = MAG3110(); //magnetometer setup
int deviation = 15;  // starting at 15 for the new sensor
int16_t x,y,z;
int oldx,oldy,oldz;
int const bufferLen = 8;
int msgid = 0;

int xbuffer[bufferLen]; // initalize circular buffer for XYZ averaging of reads.
int ybuffer[bufferLen];
int zbuffer[bufferLen];
int xaverage;
int yaverage;
int zaverage;
const String alarm_msg = "DRV";
unsigned long alarm_active = 0;
unsigned long last_checkin = 0;
unsigned long checkin_delay = 7200000;
unsigned int alarm_delay = 5000;

Adafruit_INA219 inaOne;    //INA219 current sensor setup
Adafruit_INA219 inaTwo;
float busvoltageOne = 0; //only need to know battery voltage from one sensor
float shuntvoltageOne = 0;
float loadvoltageOne = 0; //calculated from bus and shunt voltage
float currentMaOne = 0; //setting up current readings
float currentMaTwo = 0;
float maxcurrent = 1200.0; //maximum allowable current. Should be set to just slightly higher than what is required to open the gate. Any higher value is assumed to be cause by a jammed motor or gate pressing on an obstacle.

byte localAddress = 0x13; //19
byte destAddress = 0x25; //37
bool packetToParse = false;

int runtime = 12000; //slightly longer than the motors should stay to to fully open/close
bool open[4] = {1,0,1,0}; //relay states for opening/retracting both actuators
bool close[4] = {0,1,0,1};

void receiving(int packetSize)
{
  Serial.println(F("Receiving transmission"))  ;
  if (packetSize == 0) return;
  int i = 0;
  int recipient = LoRa.read();
  if (recipient != localAddress) return; //message isn't for us, quit parsing
  byte sender = LoRa.read();
  byte incomingMsgId = LoRa.read();
  byte incomingLength = LoRa.read();

  char incomingArray[incomingLength];
  while (LoRa.available())
    {
    incomingArray[i] = (char)LoRa.read();
    i++;
    }
  if (strcmp(incomingArray,"OPEN") == 0)
    {
      Serial.println(F("Retracting actuators"));
      unsigned long currentTime = millis();
      digitalWrite(relayOne, open[0]);  //opening gate, retracting actuators
      digitalWrite(relayTwo, open[1]);
      digitalWrite(relayThree, open[2]);
      digitalWrite(relayFour, open[3]);
      while (millis - currentTime < runtime)
      {
        currentMaOne = inaOne.getCurrent_mA();
        currentMaTwo = inaTwo.getCurrent_mA();
        if (currentMaOne > maxcurrent ||currentMaTwo > maxcurrent) // if max current exceeded, stop motors, pause, move if opposite direction and stop.
         {
          digitalWrite(relayOne, LOW);
          digitalWrite(relayTwo, LOW);
          digitalWrite(relayThree, LOW);
          digitalWrite(relayFour, LOW);
          delay(2000);
          digitalWrite(relayOne, close[0]);
          digitalWrite(relayTwo, close[1]);
          digitalWrite(relayThree, close[2]);
          digitalWrite(relayFour, close[3]);
         }
        delay(800); // pause before rereading current
      }
      digitalWrite(relayOne, LOW);
      digitalWrite(relayTwo, LOW);
      digitalWrite(relayThree, LOW);
      digitalWrite(relayFour, LOW);
    }
  else if (strcmp(incomingArray,"CLOSE") == 0)
    {
      Serial.println(F("Extending actuators"));
      unsigned long currentTime = millis();
      digitalWrite(relayOne, close[0]);  //closing gate, extending acutators
      digitalWrite(relayTwo, close[1]);
      digitalWrite(relayThree, close[2]);
      digitalWrite(relayFour, close[3]);
      while (millis - currentTime < runtime)
      {
        currentMaOne = inaOne.getCurrent_mA();
        currentMaTwo = inaTwo.getCurrent_mA();
        if (currentMaOne > maxcurrent ||currentMaTwo > maxcurrent) // if max current exceeded, stop motors, pause, move if opposite direction and stop.
         {
          digitalWrite(relayOne, LOW);
          digitalWrite(relayTwo, LOW);
          digitalWrite(relayThree, LOW);
          digitalWrite(relayFour, LOW);
          delay(2000);
          digitalWrite(relayOne, open[0]);
          digitalWrite(relayTwo, open[1]);
          digitalWrite(relayThree, open[2]);
          digitalWrite(relayFour, open[3]);
         }
        delay(800); // pause before rereading current

      }
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    }
  else if (strcmp(incomingArray,"STAT"))
    {
        Serial.println("status request received"); //just a placeholder
    }
  //there will be some ack/nack and collision detection, eventually
  // else if ((char*)incomingArray == "ACK")
  //   {
  //     needAck = false;
  //   }
}


void getgauss(){ //driveway magnetometer alarm reading
  xaverage = 0;
  yaverage = 0;
  zaverage = 0;
  int tempxaverage = 0; // temporary storage for the maths
  int tempyaverage = 0;
  int tempzaverage = 0;
  for (int i; i<=bufferLen; i++)
  {
    if(mag.dataReady())
    {
      mag.readMag(&x, &y, &z);
    }
    xbuffer[i] = x; // add new readings to buffer
    ybuffer[i] = y;
    zbuffer[i] = z;
    delay(30);    //lets try 8 reads every 1/2 second
  }
  for (int t; t<=bufferLen; t++)   //average everything into a single number for x, y, z
  {
    xaverage = xbuffer[t] + xaverage;
    yaverage = ybuffer[t] + yaverage;
    zaverage = zbuffer[t] + zaverage;
  }
    tempxaverage = xaverage/bufferLen;
    tempyaverage = yaverage/bufferLen;
    tempzaverage = zaverage/bufferLen;

    xaverage = abs(tempxaverage); //absolute value taken to measure difference from last reading, positive or negative is not important for our use
    yaverage = abs(tempyaverage);
    zaverage = abs(tempzaverage);
    //just printing the numbers to watch for changes while testing with magnet near sensor
    //Serial.print("Averages: "); Serial.print(xaverage); Serial.print(" "); Serial.print(yaverage); Serial.print(" "); Serial.println(zaverage);
}

void alarmmessage(){
  //Serial.println("sending");
  delay(200);
  LoRa.beginPacket();
  LoRa.write(destAddress);
  LoRa.write(localAddress);
  LoRa.write(msgid); // message ID, incremented by one at every alarm
  LoRa.write(alarm_msg.length());
  LoRa.print(alarm_msg);
  LoRa.endPacket();
  msgid++;
}

void checkin()
{
    String const online PROGMEM = "Gate sentry online";
    Serial.println(F("sending checkin"));
    delay(200);
    LoRa.beginPacket();
    LoRa.write(destAddress);
    LoRa.write(localAddress);
    LoRa.write(11); // message ID,
    LoRa.write(online.length());
    LoRa.print(online);
    LoRa.endPacket();
}

void batteryAlarm()
{
  //send low battery alert
}



void setup()
{
    Serial.begin(115200);
    Wire.begin();
    mag.initialize(); //Initializes the mag sensor - this call will set wire frequency to 400khz if you do not change it in the library!!!!
    mag.start();      //Puts the sensor in active mode
    LoRa.setPins(csPin, resetPin, irqPin);

    uint32_t currentFrequency; //why is this in the adafruit example?
    inaOne.begin();
    inaTwo.begin();

    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW); // turn off internal pullups
    pinMode(button, INPUT_PULLUP);
    pinMode(ledOne, OUTPUT);
    pinMode(ledTwo, OUTPUT);
    pinMode(relayOne, OUTPUT);
    pinMode(relayTwo, OUTPUT);
    pinMode(relayThree, OUTPUT);
    pinMode(relayFour, OUTPUT);

    if (!LoRa.begin(430E6))
  {
    while (1)
    {
      digitalWrite(ledOne, HIGH);
      delay(80);
      digitalWrite(ledOne, LOW);
      delay(80);
    }
  }
    Serial.println("Radio ready");
    LoRa.onReceive(receiving);
    LoRa.receive();
}

void loop()
{
  //check magnetometer readings every second, and check timers and battery voltage.
  //incoming radio messages are handled by callback
  getgauss();
  if (((xaverage-oldx > deviation) || (yaverage-oldy > deviation) || (zaverage-oldz > deviation)))
  {
    alarm_active = millis(); // reset alarm delay, to prevent constant triggering from the same slow vehicle
    alarmmessage();
    //LoRa.sleep();
    delay(alarm_delay); // delay before re-reading the sensor and re-zeroing magnetometer values. This will allow time for the vehicle to pass or stop near the sensor. The alarm will not continuously trip if the vehicle remains in range of the sensor. The alarm WILL trip when the vehicle moves again.
    getgauss();
    oldx = xaverage;
    oldy = yaverage;
    oldz = zaverage;
  }
  delay(500);
  if (millis() - last_checkin > checkin_delay)
  {
    checkin();
    last_checkin = millis();
  }
  delay(500);
  busvoltageOne = inaOne.getBusVoltage_V();
  shuntvoltageOne = inaOne.getBusVoltage_V();
  loadvoltageOne = busvoltageOne + (shuntvoltageOne / 1000);
  if (loadvoltageOne < 12.0)
  {
    batteryAlarm();
  }
}
