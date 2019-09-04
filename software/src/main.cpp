#include <Arduino.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <Adafruit_INA219.h>
#include <SparkFun_MAG3110.h> //don't forget to make sure the Wire.begin() call in here is for 100khz, not 400khz as is the default from sparkfun

#define csPin 10
#define resetPin 9
#define irqPin 2

#define ledOne 8
#define ledTwo 3
#define openbutton A2
#define closebutton A3

#define relayOne 4
#define relayTwo 5
#define relayThree 6
#define relayFour 7

//%%%%%%%%%%%%%%%%%%% TODO %%%%%%%%%%%%%%%%%%%%%%%
//more serial debug

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
unsigned long last_battery_check = 0;
unsigned long checkin_delay = 7200000;
unsigned int alarm_delay = 5000;

Adafruit_INA219 inaOne;    //INA219 current sensor constructors
Adafruit_INA219 inaTwo;

float busvoltageOne = 0; //only need to know battery voltage from one sensor
float shuntvoltageOne = 0;
float loadvoltageOne = 0; //calculated from bus and shunt voltage in function getLoadVoltage()
float busvoltageTwo = 0;
float shuntvoltageTwo = 0;
float loadvoltageTwo = 0;
float currentMaOne = 0; //setting up current readings
float currentMaTwo = 0;
float maxcurrent = 1000.0; //maximum allowable current. Should be set to just slightly higher than what is required to open the gate. Any higher value is assumed to be cause by a jammed motor or gate pressing on an obstacle. Is in milliamps

byte localAddress = 0x13; //19
byte destAddress = 0x25; //37
bool packetToParse = false;

int runtime = 30000; //slightly longer than the motors should stay to to fully open/close. 16" acutators take about 28 seconds from limit to limit
static const bool open[4] = {1,0,1,0}; //relay states for opening/retracting both actuators
static const bool close[4] = {0,1,0,1};
bool closeflag = false; // flag to determine if the gate is supposed to close or not
bool openflag = false; // flag to determine if the gate is supposed to open or not
bool gateisopen = false; // system needs to start with gate closed. answers the question 'is the gate open'


void getLoadVoltage() // returns nothing, just update global variables.
  {
    busvoltageOne = inaOne.getBusVoltage_V();
    shuntvoltageOne = inaOne.getShuntVoltage_mV();
    loadvoltageOne = busvoltageOne + (shuntvoltageOne / 1000);
    busvoltageTwo = inaTwo.getBusVoltage_V();
    shuntvoltageTwo = inaTwo.getShuntVoltage_mV();
    loadvoltageTwo = busvoltageTwo + (shuntvoltageTwo / 1000);
  }

void faultmessage(String faultType) //called when experiencing fault. "fault1" = no bus voltage , "fault2" = no bus current , "fault3" = overcurrent
  {
    delay(100);
    LoRa.beginPacket();
    LoRa.write(destAddress);
    LoRa.write(localAddress);
    LoRa.write(msgid);
    LoRa.write(faultType.length());
    LoRa.print(faultType);
    LoRa.endPacket();
    msgid++;
  }

void openGate()
{
  if (gateisopen == true) { // gate is already open
    return;
  }
  Serial.println(F("Retracting actuators"));
  unsigned long currentTime = millis();
  digitalWrite(relayOne, open[0]);  //opening gate, retracting actuators
  digitalWrite(relayTwo, open[1]);
  digitalWrite(relayThree, open[2]);
  digitalWrite(relayFour, open[3]);
  delay(50); //short delay for motors to spin up?
  getLoadVoltage();
  currentMaOne = inaOne.getCurrent_mA();
  currentMaTwo = inaTwo.getCurrent_mA();
  if (loadvoltageOne < 10 || loadvoltageTwo < 10) //no voltage on actuator lines mean blown fuses
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    Serial.println("no volt fault");
    faultmessage("fault1");
  }
  else if (currentMaOne < 200 || currentMaTwo < 200) //no current on actuator lines means motor disconnected
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    Serial.println("no crnt fault");
    faultmessage("fault2");
  }
  else if (currentMaOne > maxcurrent || currentMaTwo > maxcurrent) //overcurrent as soon as things are moving means something is very jammed. Just stop all motors.
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    Serial.println("overcurrent");
    faultmessage("fault3");
  }
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
      delay(250);
      faultmessage("fault3");
      Serial.println("overcurrent");
      currentMaOne = inaOne.getCurrent_mA();
      currentMaTwo = inaTwo.getCurrent_mA();
      if (currentMaOne > maxcurrent ||currentMaTwo > maxcurrent) // if max current exceeded while reversing direction, just stop everything and halt until reset.
        {
          digitalWrite(relayOne, LOW);
          digitalWrite(relayTwo, LOW);
          digitalWrite(relayThree, LOW);
          digitalWrite(relayFour, LOW);
          while (true)
            {
              gateisopen = false;
              openflag = false;
              closeflag = false;
              digitalWrite(ledOne, HIGH);
              digitalWrite(ledTwo, LOW);
              delay(333);
              digitalWrite(ledOne, LOW);
              digitalWrite(ledTwo, HIGH);
            }
        }
     }
    delay(500); // pause before rereading current
  }
  digitalWrite(relayOne, LOW);
  digitalWrite(relayTwo, LOW);
  digitalWrite(relayThree, LOW);
  digitalWrite(relayFour, LOW);
  openflag = false;
  gateisopen = true;
}

void closeGate()
{
  if (gateisopen == false) { // gate is already closed
    return;
  }
  Serial.println(F("Extending actuators"));
  unsigned long currentTime = millis();
  digitalWrite(relayOne, close[0]);  //closing gate, extending acutators
  digitalWrite(relayTwo, close[1]);
  digitalWrite(relayThree, close[2]);
  digitalWrite(relayFour, close[3]);
  delay(50); //short delay for motors to spin up?
  getLoadVoltage();
  currentMaOne = inaOne.getCurrent_mA();
  currentMaTwo = inaTwo.getCurrent_mA();
  if (loadvoltageOne < 10 || loadvoltageTwo < 10) //no voltage on actuator lines mean blown fuses
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    faultmessage("fault1");
    Serial.println("no volt fault");
  }
  else if (currentMaOne < 200 || currentMaTwo < 200) //no current on actuator lines means motor disconnected
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    faultmessage("fault2");
    Serial.println("no crnt fault");
  }
  else if (currentMaOne > maxcurrent || currentMaTwo > maxcurrent) //overcurrent as soon as things are moving means something is very jammed. Just stop all motors.
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    Serial.println("overcurrent");
    faultmessage("fault3");
  }
  while (millis - currentTime < runtime) // run motors for 30 seconds, checking every 400ms for a motor overcurrent condition
  {
    currentMaOne = inaOne.getCurrent_mA();
    currentMaTwo = inaTwo.getCurrent_mA();
    if (currentMaOne > maxcurrent || currentMaTwo > maxcurrent) // if max current exceeded, stop motors, pause, move if opposite direction and stop.
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
      delay(250);
      faultmessage("fault3");
      Serial.println("overcurrent");
      currentMaOne = inaOne.getCurrent_mA();
      currentMaTwo = inaTwo.getCurrent_mA();
      if (currentMaOne > maxcurrent || currentMaTwo > maxcurrent) // if max current exceeded while reversing direction, just stop everything and halt until reset.
        {
          digitalWrite(relayOne, LOW);
          digitalWrite(relayTwo, LOW);
          digitalWrite(relayThree, LOW);
          digitalWrite(relayFour, LOW);
          while (true)
            {
              gateisopen = true;
              closeflag = false;
              openflag = false;
              digitalWrite(ledOne, HIGH);
              digitalWrite(ledTwo, LOW);
              delay(333);
              digitalWrite(ledOne, LOW);
              digitalWrite(ledTwo, HIGH);
            }
        }
    delay(400); // pause before rereading current

    }
  }
  digitalWrite(relayOne, LOW);
  digitalWrite(relayTwo, LOW);
  digitalWrite(relayThree, LOW);
  digitalWrite(relayFour, LOW);
  closeflag = false;
  gateisopen = false;
}

// void packetReceived(int packetSize) //ISR to set flag for parsing a packet.
// {
//   packetToParse = true;
// }

void receiving(int packetSize) // crap, is this an interrupt function? so millis and delay would not work?
{
  //detachInterrupt(irqPin);
  Serial.println(F("Receiving transmission"));
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
      Serial.println("open req");
      openflag = true;
      closeflag = false;
    }
  else if (strcmp(incomingArray,"CLOSE") == 0)
    {
      Serial.println("close req");
      closeflag = true;
      openflag = false;
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
//  attachInterrupt(uint8_t, void (*)(), int mode)
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

void alarmmessage()
{
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

void batteryCheck()
{
  //send low battery alert
  getLoadVoltage();
  if (loadvoltageOne < 12 && loadvoltageTwo <12)
    {
      LoRa.beginPacket();
      LoRa.write(destAddress);
      LoRa.write(localAddress);
      LoRa.write(msgid); // message ID, incremented by one at every alarm
      LoRa.write(4);
      LoRa.print("batt");
      LoRa.endPacket();
      msgid++;
    }
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
    pinMode(openbutton, INPUT_PULLUP);
    pinMode(closebutton, INPUT_PULLUP);
    pinMode(ledOne, OUTPUT);
    pinMode(ledTwo, OUTPUT);
    pinMode(relayOne, OUTPUT);
    pinMode(relayTwo, OUTPUT);
    pinMode(relayThree, OUTPUT);
    pinMode(relayFour, OUTPUT);


    //some startup debug stuff
    getLoadVoltage();
    Serial.print("volt one:"); Serial.println(loadvoltageOne);
    Serial.print("volt two:"); Serial.println(loadvoltageTwo);


    if (!LoRa.begin(430E6))
  {
    Serial.println("radio init failed");
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

    //getgauss(); //get and zero a magnetometer reading
}

void loop()
{
  //check magnetometer readings every second, and check timers and battery voltage.
  //incoming radio messages are handled by an ISR that sets

  // getgauss();
  // if (((xaverage-oldx > deviation) || (yaverage-oldy > deviation) || (zaverage-oldz > deviation)))
  // {
  //   Serial.println("magnetic alarm");
  //   alarm_active = millis(); // reset alarm delay, to prevent constant triggering from the same slow vehicle
  //   alarmmessage();
  //   //LoRa.sleep();
  //   delay(alarm_delay); // delay before re-reading the sensor and re-zeroing magnetometer values. This will allow time for the vehicle to pass or stop near the sensor. The alarm will not continuously trip if the vehicle remains in range of the sensor. The alarm WILL trip when the vehicle moves again.
  //   getgauss();
  //   oldx = xaverage;
  //   oldy = yaverage;
  //   oldz = zaverage;
  // }
  delay(200);
  if (millis() - last_checkin > checkin_delay)
  {
    checkin();
    last_checkin = millis();
  }
  if (openflag == true)
  {
    openGate();
  }
  if (closeflag == true)
  {
    closeGate();
  }
  if (millis() - last_battery_check > 600000) //check battery level every 10 minutes
  {
    batteryCheck();
  }
  if (digitalRead(openbutton) == LOW) //copypasting the code from lora function 'receiving'. with the interrupts that might have to be changed to checking for a flag of some kind. fortesting, just indicate things are running OK
   {
     openGate();
   }
  if (digitalRead(closebutton) == LOW)
  {
    closeGate();
  }
}
