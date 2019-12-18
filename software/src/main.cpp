#include <Arduino.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <Adafruit_INA219.h>
#include <SparkFun_MAG3110.h> //don't forget to make sure the Wire.begin() call in here is for 100khz, not 400khz as is the default from sparkfun

//CHANGES MADE IN THIS VERSION: since installing- turned off heartbeat led, change startup curren limit from 1400ma to 1600, change max current while running from 1000 to 1300ma

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
unsigned long checkin_delay = 14000000;
unsigned int alarm_delay = 5000;

Adafruit_INA219 inaOne(0x40);    //INA219 current sensor constructors
Adafruit_INA219 inaTwo(0x41);

float busvoltageOne = 0; //only need to know battery voltage from one sensor
float shuntvoltageOne = 0;
float loadvoltageOne = 0; //calculated from bus and shunt voltage in function getLoadVoltage()
float busvoltageTwo = 0;
float shuntvoltageTwo = 0;
float loadvoltageTwo = 0;
float currentMaOne = 0; //setting up current readings
float currentMaTwo = 0;
float maxcurrent = 1300.0; //maximum allowable current. Should be set to just slightly higher than what is required to open the gate. Any higher value is assumed to be cause by a jammed motor or gate pressing on an obstacle. Is in milliamps

byte localAddress = 0x13; //19
byte destAddress = 0x25; //37
bool packetToParse = false;

unsigned long runtime = 32000; //slightly longer than the motors should stay to to fully open/close. 16" acutators take about 28 seconds from limit to limit, plus some delays to stagger the startup current
bool open[4] = {1,0,1,0}; //relay states for opening/retracting both actuators
bool close[4] = {0,1,0,1};
bool closeflag = false; // flag to determine if the gate is supposed to close or not
bool openflag = false; // flag to determine if the gate is supposed to open or not
bool gateisopen = false; // system needs to start with gate open [actuators retracted]. answers the question 'is the gate open'


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
    delay(100);
    LoRa.receive();
  }

void checkin()
{
    String const online PROGMEM = "Gate sentry online";
    delay(200);
    LoRa.beginPacket();
    LoRa.write(destAddress);
    LoRa.write(localAddress);
    LoRa.write(11); // message ID,
    LoRa.write(online.length());
    LoRa.print(online);
    LoRa.endPacket();
    msgid++;
    delay(100);
    LoRa.receive();
}

void openGate()
{
  unsigned long currentTime = millis(); //when the motors started running
  if (gateisopen == true)
  { // gate is already open
    //Serial.println("already open");
    openflag = false;
    closeflag = false;
    gateisopen = false;
    return;
  }
  bool errorFlag = false;
  //Serial.println(F("Retracting actuators"));
  digitalWrite(relayOne, open[0]);  //opening gate, retracting actuators
  digitalWrite(relayTwo, open[1]);
  delay(200); //slightly offset startup current by not starting motors at exact same time
  digitalWrite(relayThree, open[2]);
  digitalWrite(relayFour, open[3]);
  delay(1000); //short delay for motors to spin up?
  getLoadVoltage();
  currentMaOne = inaOne.getCurrent_mA();
  currentMaTwo = inaTwo.getCurrent_mA();
  delay(10);
  //Serial.print("current ma:"); Serial.println(currentMaOne);
  //Serial.print("current ma:"); Serial.println(currentMaTwo);
  if (loadvoltageOne < 6 || loadvoltageTwo < 6) //no voltage on actuator lines mean blown fuses
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    delay(100);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    //Serial.println("no volt fault");
    faultmessage("fault1");
    openflag = false; //setitng these equal to false will 'zero out' the flags if any change occured due to an interrupt for the radio setting one to true. Prevents opener from acting on multiple open/close commands recieved while the gate is in motion
    closeflag = false;
    gateisopen = true;
    return;
  }
  else if (currentMaOne < 100 || currentMaTwo < 100) //no current on actuator lines means motor disconnected
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    delay(100);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    //Serial.println("no crnt fault");
    faultmessage("fault2");
    openflag = false; //setitng these equal to false will 'zero out' the flags if any change occured due to an interrupt for the radio setting one to true. Prevents opener from acting on multiple open/close commands recieved while the gate is in motion
    closeflag = false;
    gateisopen = true;
    return;
  }
  else if (currentMaOne > 1600 || currentMaTwo > 1600) //overcurrent as soon as things are moving means something is very jammed. Just stop all motors.
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    delay(100);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    //Serial.println("overcurrent");
    faultmessage("fault3");
    openflag = false; //setitng these equal to false will 'zero out' the flags if any change occured due to an interrupt for the radio setting one to true. Prevents opener from acting on multiple open/close commands recieved while the gate is in motion
    closeflag = false;
    //gateisopen = false;
    return;
  }
  while (millis() - currentTime < runtime)
  {
    currentMaOne = inaOne.getCurrent_mA();
    currentMaTwo = inaTwo.getCurrent_mA();
    //Serial.print("crnt 1: "); Serial.println(currentMaOne);
    //Serial.print("crnt 2: "); Serial.println(currentMaTwo);
    if (currentMaOne > maxcurrent ||currentMaTwo > maxcurrent) // if max current exceeded, stop motors, pause, move if opposite direction and stop.
    {
        currentTime += 2500; // add 2.5 seconds to compensate for delays on stopping and reversing. Will run until actuators are fullly open
        //Serial.println("overcurrent");
        digitalWrite(relayOne, LOW);
        digitalWrite(relayTwo, LOW);
        delay(200);
        digitalWrite(relayThree, LOW);
        digitalWrite(relayFour, LOW);
        delay(1500);
        digitalWrite(relayOne, close[0]);
        digitalWrite(relayTwo, close[1]);
        delay(200); //slightly offset startup current by not starting motors at exact same time
        digitalWrite(relayThree, close[2]);
        digitalWrite(relayFour, close[3]);
        delay(100);
        faultmessage("fault3");
        errorFlag = true;

        // if (currentMaOne > maxcurrent || currentMaTwo > maxcurrent) // if max current exceeded while reversing direction, just stop everything and wait.
        // {
        //   Serial.println("double overcurrent");
        //   digitalWrite(relayOne, LOW);
        //   digitalWrite(relayTwo, LOW);
        //   delay(200);
        //   digitalWrite(relayThree, LOW);
        //   digitalWrite(relayFour, LOW);
        //   delay(200);
        // }
      //}
      // digitalWrite(relayOne, LOW);
      // digitalWrite(relayTwo, LOW);
      // delay(200);
      // digitalWrite(relayThree, LOW);
      // digitalWrite(relayFour, LOW);
    }
    delay(500); // pause before rereading current
  }
  digitalWrite(relayOne, LOW);
  digitalWrite(relayTwo, LOW);
  delay(100);
  digitalWrite(relayThree, LOW);
  digitalWrite(relayFour, LOW);
  openflag = false; //setitng these equal to false will 'zero out' the flags if any change occured due to an interrupt for the radio setting one to true. Prevents opener from acting on multiple open/close commands recieved while the gate is in motion
  closeflag = false;
  gateisopen = true;
  if (errorFlag == true) gateisopen = false; //if gate reversed, it's closed now, not opener
  errorFlag = false;
  LoRa.receive();
}

void closeGate()
{
  unsigned long currentTime = millis(); //when the motors started running
  if (gateisopen == false)
  { // gate is already closed
    //Serial.println("already closed");
    closeflag = false;
    openflag = false;
    gateisopen = true; // set to true so that if control was started with gate in wrong position, is will reset and try to open on second try.
    return;
  }
  bool errorFlag = false;
  //Serial.println(F("Extending actuators"));
  digitalWrite(relayOne, close[0]);  //closing gate, extending acutators
  digitalWrite(relayTwo, close[1]);
  delay(200); //slightly offset startup current by not starting motors at exact same time
  digitalWrite(relayThree, close[2]);
  digitalWrite(relayFour, close[3]);
  delay(1000); //short delay for motors to spin up?
  getLoadVoltage();
  currentMaOne = inaOne.getCurrent_mA();
  currentMaTwo = inaTwo.getCurrent_mA();
  delay(10);
  //Serial.print("current ma:"); Serial.println(currentMaOne);
  //Serial.print("current ma:"); Serial.println(currentMaTwo);
  if (loadvoltageOne < 6 || loadvoltageTwo < 6) //no voltage on actuator lines mean blown fuses
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    delay(100);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    faultmessage("fault1");
    //Serial.println("no volt fault");
    closeflag = false; //setitng all these equal to false will 'zero out' the flags if any change occured due to an interrupt for the radio setting one to true. Prevents opener from acting on multiple open/close commands recieved while the gate is in motion
    openflag = false;
    gateisopen = false;
    return;
  }
  else if (currentMaOne < 100 || currentMaTwo < 100) //no current on actuator lines means motor disconnected
  {
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    delay(100);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    faultmessage("fault2");
    //Serial.println("no crnt fault");
    closeflag = false; //setitng all these equal to false will 'zero out' the flags if any change occured due to an interrupt for the radio setting one to true. Prevents opener from acting on multiple open/close commands recieved while the gate is in motion
    openflag = false;
    //gateisopen = false;
    return;
  }
  else if (currentMaOne > 1400 || currentMaTwo > 1400) //overcurrent as soon as things are moving means something is very jammed. Just stop all motors.
  {
    //Serial.println("overcurrent");
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    delay(100);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);
    faultmessage("fault3");
    closeflag = false; //setitng all these equal to false will 'zero out' the flags if any change occured due to an interrupt for the radio setting one to true. Prevents opener from acting on multiple open/close commands recieved while the gate is in motion
    openflag = false;
    gateisopen = false;
    return;
  }
  while (millis() - currentTime < runtime) // run motors for 30 seconds, checking every 400ms for a motor overcurrent condition
  {
    currentMaOne = inaOne.getCurrent_mA();
    currentMaTwo = inaTwo.getCurrent_mA();
    //Serial.print("crnt 1: "); Serial.println(currentMaOne);
    //Serial.print("crnt 2: "); Serial.println(currentMaTwo);
    if (currentMaOne > maxcurrent || currentMaTwo > maxcurrent) // if max current exceeded, stop motors, pause, move in opposite direction and stop.
    {
        currentTime += 2500; // add 2.5 seconds to currentTime to compenstate for stop/reverse delays
        //Serial.println("overcurrent");
        digitalWrite(relayOne, LOW);
        digitalWrite(relayTwo, LOW);
        delay(200);
        digitalWrite(relayThree, LOW);
        digitalWrite(relayFour, LOW);
        delay(1500);
        digitalWrite(relayOne, open[0]);
        digitalWrite(relayTwo, open[1]);
        delay(200); //slightly offset startup current by not starting motors at exact same time
        digitalWrite(relayThree, open[2]);
        digitalWrite(relayFour, open[3]);
        delay(100);
        faultmessage("fault3");
        errorFlag = true;
      //   currentMaOne = inaOne.getCurrent_mA();
      //   currentMaTwo = inaTwo.getCurrent_mA();
        // if (currentMaOne > maxcurrent || currentMaTwo > maxcurrent) // if max current exceeded while reversing direction, just stop everything and halt until reset.
        // {
        //   Serial.println("double overcurrent");
        //   digitalWrite(relayOne, LOW);
        //   digitalWrite(relayTwo, LOW);
        //   delay(200);
        //   digitalWrite(relayThree, LOW);
        //   digitalWrite(relayFour, LOW);
        //   delay(250);
        // }
      //}
      // digitalWrite(relayOne, LOW);
      // digitalWrite(relayTwo, LOW);
      // delay(200);
      // digitalWrite(relayThree, LOW);
      // digitalWrite(relayFour, LOW);
    }
    delay(500); // pause before rereading current
  }
  digitalWrite(relayOne, LOW);
  digitalWrite(relayTwo, LOW);
  delay(200);
  digitalWrite(relayThree, LOW);
  digitalWrite(relayFour, LOW);
  closeflag = false; //setitng all these equal to false will 'zero out' the flags if any change occured due to an interrupt for the radio setting one to true. Prevents opener from acting on multiple open/close commands recieved while the gate is in motion
  openflag = false;
  gateisopen = false;
  if (errorFlag == true) gateisopen = true; //if gate reversed, it's open now, not closed
  errorFlag = false;
  LoRa.receive();
}

// void packetReceived(int packetSize) //ISR to set flag for parsing a packet.
// {
//   packetToParse = true;
// }

void receiving(int packetSize) // crap, is this an interrupt function?
{
  if (packetSize == 0) return;
  int i = 0;
  int recipient = LoRa.read();
  if (recipient != localAddress) return; //message isn't for us, quit parsing
  byte sender = LoRa.read();
  //Serial.print("sndr: "); Serial.println(sender);
  byte incomingMsgId = LoRa.read();
  byte incomingLength = LoRa.read();

  char incomingArray[incomingLength];
  while (LoRa.available())
  {
    incomingArray[i] = (char)LoRa.read();
    i++;
  }
  // Serial.print("incomingArray = "); Serial.write(incomingArray, incomingLength);Serial.print(" : "); Serial.println(incomingLength);
  // Serial.println(strlen(incomingArray));
  // Serial.println(strncmp(incomingArray, "OPEN",4));
  // Serial.println(strncmp(incomingArray, "CLOSE",5));
  // Serial.println(strncmp(incomingArray, "STAT",4));
  if (strncmp(incomingArray,"OPEN",4) == 0) // strcmp never works right. maybe there's a trailing newline issue? strncmp and defining the expected array length seems to work fine
  {
    //Serial.println("open req");
    openflag = true;
    closeflag = false;
  }
  else if (strncmp(incomingArray,"CLOSE",5) == 0)
  {
    //Serial.println("close req");
    closeflag = true;
    openflag = false;
  }
  else if (strncmp(incomingArray,"TOG",3) == 0)
  {
    if (gateisopen == true)
    {
      closeflag = true;
      openflag = false;
    }
    else
    {
      closeflag = false;
      openflag = true;
    }
  }
  else if (strncmp(incomingArray,"STAT",4) == 0)
  {
    //Serial.println("status req");
    checkin();
  }
  //else
  //{
    //Serial.println("rx unrec");
  //}
    // if (openflag)
    // {
    //   // put lora status message ehre
    // }
    // else if (closeflag)
    // {
    //   // put lora status message here
    // }
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
      delay(100);
      LoRa.receive();
    }
  last_battery_check = millis();
}



void setup()
{
    Serial.begin(57600);
    Serial.println("Software v1.2"); // v2.0 should include new ack/nack setup
    Wire.begin();
    //mag.initialize(); //Initializes the mag sensor - this call will set wire frequency to 400khz if you do not change it in the library!!!!
    //mag.start();      //Puts the sensor in active mode
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
    digitalWrite(ledOne, HIGH);
    digitalWrite(ledTwo, HIGH);
    delay(2000);
    digitalWrite(ledOne, LOW);
    digitalWrite(ledTwo, LOW);
    digitalWrite(relayOne, LOW);
    digitalWrite(relayTwo, LOW);
    digitalWrite(relayThree, LOW);
    digitalWrite(relayFour, LOW);

    if (!LoRa.begin(430E6))
  {
    Serial.println("radio init failed");
    while (1)
    {
      digitalWrite(ledOne, HIGH);
      delay(150);
      digitalWrite(ledOne, LOW);
      delay(150);
    }
  }
    //Serial.println("Radio ready");
    LoRa.onReceive(receiving);
    LoRa.receive();
    //getgauss(); //get and zero a magnetometer reading
}

void loop()
{
  //digitalWrite(ledOne, HIGH); // heartbeat blink for testing. will remove later
  delay(250);
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
  if (millis() - last_checkin > checkin_delay) //
  {
    //Serial.println("check in");
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
    //Serial.println("battery check");
    batteryCheck();
  }
  if (digitalRead(openbutton) == LOW)
   {
     openflag = true; //override flags to attempt opening or closing when button is pushed
     gateisopen = false;
     openGate();
   }
  if (digitalRead(closebutton) == LOW)
  {
    closeflag = true; //override flags to attempt opening or closing when button is pushed
    gateisopen = true;
    closeGate();
  }
  //digitalWrite(ledOne, LOW);
  delay(250); //delay for led blink
}
