/**************************************************************************
  Espresso_timer_code
  Version 1.0.0
  Comments to follow...
  8/2020
 **************************************************************************/

//include files

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h> //serial protocol communication...do I need this?
#include <Wire.h> //i2c devices
#include <Adafruit_GFX.h> //adafruit oled screen
#include <Adafruit_SSD1306.h> // oled screen
#include <bearSplash.h> //custom splash screen, probably should have renamed this
//#include "Adafruit_MPRLS.h" //ported pressure sensor
#include <Adafruit_Sensor.h> //adafruit generic sensor class
#include <Adafruit_BMP085_U.h> //bmp pressure sensor
#include <Encoder.h> //encoder library. best one i've found
// include the sleep controls
#include <avr/sleep.h>
//#include "DRV8825.h" //stepper driver file for pololu boards

//Global Constants particular to objects//
#define REQUIRESALARMS false

//oled parameters
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     9 // Reset pin # (or -1 if sharing Arduino reset pin)
//stepper motor parameters
#define MOTOR_STEPS 200
#define RPM 400
#define MICROSTEPS 2
#define DIR 10
#define STEP 11
#define SLEEP 12
#define MOTOR_ACCEL 1000
#define MOTOR_DECEL 2000
//custom constant for debug. hardly used in the code
#define DEBUG 1
//#define ENCODER_OPTIMIZE_INTERRUPTS

//Global variables

//flags
static volatile bool buttonFlag = false; //button interrupt pin, volatile for the ISR
static bool powerOnFlag = false;//status flag for machine power controlled via a transistor/relay pair
static bool brewFlag = false;//status flag for brewing. Keep cycling through checking sensors until ready to brew
static bool tempFlag = false;
static bool waterSense = true;
//pin definitions
static const int buttonPin = 2;//button
static const int encodeA = 3;//encoder pin
static const int encodeB = 4;//encoder pin2 is not set to an interrupt (needed one for the button)
static const int powerPin = 8;//power relay pin, driven via an NPN
static const int tempPin = A0;//heater sensor, Low when heater is turned off (at temp)
static const int valvePin = 7;//valve sensor, High when valve is set correctly
static const int brewPin = 5;//brew relay pin, driven via an NPN
//static const int debugPin = A0;

//display constants
/*display is set up in three sections
  ----------------|             |
  Current setting | Water Level |
  ----------------|             |
  staus message   |             |

*/
static const int rectWidth = 26; //width of the water level block
static const int rectHeight = SCREEN_HEIGHT; //height of water level block
static const int rectThick = 2; //thickness of the vertical lines

//brew constants
static const int maxTime = 35; //max time we can brew a cup
static const int twoCups = 40; //used for the end of the menu...Grinding
static const int steepDelay = 3;//how long to run the pump for steeping (seconds)
static const long steepDuration = 5; //how long to let the grains soak (seconds)
static const long cupTurns = 600000;// How long to grind for one cup
static const int brewTemp = 85; //temperature to brew at in degrees celsius

//water level constants
static const float psiToInch = 27.7076;//obvious
static const float minWaterLevel = 0.4;//in inches
static const float maxWaterLevel = 8.4;//in inches

String statusMsg; //scroll status function passes this around. easier to be global

//keep track of how long the machine hasn't been used
static unsigned long inactiveTime; //measured inactivity value
const unsigned long inactiveThreshold = 300000; //set inactivity threshhold for 5 minutes. Turn off after

//function prototype for default value functions
//waterLevel converts the difference between pressure sensors to a bar graph
//reading is used as a debug value, readLevel allows for the animation at startup
int waterLevel(float reading = 20.0, bool readLevel = true);
//set the current setting message based on encoder position
//resetStatus is default for resetting the status message in case the setting wasn't actually changed
//ie after a sub-routine completion (brew, grind, etc)
void setDisplayType(long dialPos, bool resetStatus = false);
short tempC = 0;

//object definitions
//Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
//DRV8825 stepper(MOTOR_STEPS, DIR, STEP, SLEEP);//, MODE0, MODE1, MODE2);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Encoder myEnc(encodeA, encodeB);
OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);
DeviceAddress insideThermometer = {0x28};

void setup() {
  //Serial.begin(9600);
  //set pin directions
  unsigned long tempReadTime;
  pinMode(powerPin, OUTPUT);
  pinMode(tempPin, INPUT);
  pinMode(valvePin, INPUT);//needs pulldown resistor
  //pinMode(debugPin, INPUT);//pulldown
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(encodeA, INPUT_PULLUP);
  pinMode(encodeB, INPUT_PULLUP);
  pinMode(brewPin, OUTPUT);
  //pinMode(speedPin, INPUT);
  digitalWrite(brewPin, LOW); //don't brew
  attachInterrupt(0, buttonPress, LOW);//low side button interrupt

  //stepper motor, accel profile needed because of high start torque load
  /*  stepper.begin(RPM, MICROSTEPS);
    stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
    stepper.disable();//disable or draw heaps of power while at rest
  */
  //check for display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    //Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  //banner display, splash screen is pre-loaded from splash.h (custom is bearspresso logo)
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();
  delay(1500);
  display.clearDisplay();
  display.setTextSize(2);
  //check that pressure sensors are present
  /* REMOVED MPR Sensor
    if (! mpr.begin()) {
      //Serial.print("no MPR");
      //for (;;); // Don't proceed, loop forever

      display.setCursor(0, 0);
      display.println("! MPR");
      display.display();
      delay(1000);

      waterSense = false;
    }
  */
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    //Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    //display.setCursor(0, 0);
    display.println("! BMP");
    display.display();
    //delay(1000);
    waterSense = false;
  }
  tempSensor.begin();
  // Must be called before search()
  oneWire.reset_search();
  // assigns the first address found to insideThermometer
  oneWire.search(insideThermometer);
  tempSensor.setResolution(insideThermometer, 9);
  tempSensor.setWaitForConversion(false);

  tempSensor.requestTemperatures();
  tempReadTime = millis();
  while (millis() - tempReadTime < 1000) {
    // && !(tempSensor.getWaitForConversion()));
    delay(1);
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  tempC = (short) tempSensor.getTempC(insideThermometer);
  display.print(tempC);
  display.print(char(247));
  display.print("C");
  display.display();
  delay(2000);

  display.setTextSize(1);

  //run the bearSpresso banner, and water level animation
  startupScreen();
  setScreen();//set the layout
  if (waterSense) {
    waterLevel(getPressure());//set the actual water pressure graph
  }

  //test the pressure sensors
  //  while(1){
  //    float pressure_inch = getPressure();
  //    waterLevel(pressure_inch);
  //  }
  //let's shut everything down
  setStatus("OFF");
  myEnc.write(2);
  sleepTime();//put the uC to sleep
  /*
    while (0) {
      display.clearDisplay();
      display.setCursor(0, 0);
      tempSensor.requestTemperatures();
      delay(10);
      tempC = tempSensor.getTempC(insideThermometer);
      display.print(tempC);
      display.print(char(247));
      display.print("C");
      display.display();
      delay(2000);
    }
  */
}

void loop() {
  unsigned long tempReadTime;
  long newPosition;//store the encoder position, not sure why I used a long.
  static long oldPosition = 2;//keep the old position to compare against the current reading
  static bool idleFlag = false;//have things not changed for 30 seconds?
  inactiveTime = millis();//how long have things not changed?
  //as long as the button hasn't been pressed, read from the encoder
  while (!buttonFlag) {
    newPosition = myEnc.read();
    //take action if the reading has changed
    if (newPosition != oldPosition) {
      inactiveTime = millis();//restart the inactive counter
      //added this check to keep the encoder from bouncing around
      //I *THINK* the display calls take long enough that counts are being missed
      //The encoder does very poorly if it misses a count
      //3 seems to be stable while allowing fast scrolling
      if (abs(newPosition - oldPosition) > 3) {
        myEnc.write(oldPosition);
      }
      else {
        setDisplayType(newPosition);//set the display based on the new reading
        oldPosition = newPosition;//store the new reading
      }
    }
    //if we've been inactive for 5 minutes, turn the machine off
    else if ((inactiveTime + inactiveThreshold) < millis()) {
      setStatus("OFF");
      sleepTime();
    }
    //if inactive for 30 seconds scroll the status message
    else if ((inactiveTime + 30000) < millis()) {
      scrollStatus(true);//each call to this shifts the message by one character (direction determined in routine)
      //only need to set this once. Checkng first is slightly faster (not that it matters)
      if (!idleFlag) {
        idleFlag = true;
      }
    }
    //if previously inactive (scrolled status), stop scrolling
    else if (idleFlag) {
      scrollStatus(false);//stop scrolling
      idleFlag = false;//not idle anymore!
      setDisplayType(oldPosition, true);//need to reset the status message to center, use true to override the default
    }
  }
  //if the button was pressed, take some action
  if (buttonFlag) {
    detachInterrupt(0);//make sure we don't read any extra buttons presses
    pinMode(encodeA, OUTPUT); //need to disable the encoder interrupt
    digitalWrite(encodeA, LOW); //ditto^
    //off position, turn machine off
    if (oldPosition <= 1) {
      setStatus("OFF");
      sleepTime();
    }
    //far right of the menu selections, grind some beans!
    else if (oldPosition >= maxTime * 2) {
      if (oldPosition <= maxTime * 2 + 2) {
        grind(1);//grind one cup
      }
      else {
        grind(2);//grind two cups
      }
    }
    //between off and grind, let's brew some coffee
    else if (oldPosition > 1 && oldPosition < maxTime * 2) {
      //keep cycling through these checks until all pass
      while (!brewFlag) {
        if (waterSense) {
          float pressure_inch = getPressure();
          //check the water level...bypassed until sensors work better
          //minWaterLevel should be dependant on brewtime, need to write this too
          if (pressure_inch < minWaterLevel || DEBUG) {
            setStatus("WATER");
            //keep checking the water level until enough to brew
            while (pressure_inch < minWaterLevel) {
              float old_pressure = pressure_inch; //only update the screen if the level is changing
              pressure_inch = getPressure();
              scrollStatus(true);//keep scrolling the status
              if (abs(pressure_inch - old_pressure) > 0.25) {
                waterLevel(pressure_inch);
                old_pressure = pressure_inch;//I'm confused about why I've used this twice. need to check alogrithm
              }
            }
          }
        }
        scrollStatus(false);//stop scrolling
        setStatus("CLEAR");//clear the status
        //check the valve is turned correctly
        if (!digitalRead(valvePin)) {
          setStatus("VALVE-RIGHT");
          while (!digitalRead(valvePin)) {
            scrollStatus(true);
            delay(10);//otherwise this will fly
          }
          scrollStatus(false);
          setStatus("CLEAR");
        }
        while (!tempFlag) {
          tempSensor.requestTemperatures();
          tempReadTime = millis();
          while (millis() - tempReadTime < 1000) {
            scrollStatus(true);
            delay(10);
          }
          tempC = (short) tempSensor.getTempC(insideThermometer);
          statusMsg = "HEATING: " + String(tempC) + char(247) + "C";
          if (tempC > brewTemp) {
            coolDown();
          }
          else if (tempC == brewTemp) {
            tempFlag = true;
          }
        }
        scrollStatus(false);
        setStatus("CLEAR");
        //check all of these again...maybe I changed the valve position while adding water, so don't brew
        if (digitalRead(valvePin)) {
          if (waterSense) {
            if (getPressure() > minWaterLevel) {
              brewFlag = true;
            }
          }
          else {
            brewFlag = true;
          }
        }
      }
      startBrew(oldPosition / 2);//let's brew some coffee, the encoder is half count so need to divide by two
      setStatus("COMPLETE");
      delay(3000);//wait a bit for effect
      brewFlag = false;//reset the brew flag
      tempFlag = false;// reset the temperature flag
    }
    setDisplayType(oldPosition, true);//reset the message and status blocks
    buttonFlag = false;//reset the button flag
    attachInterrupt(0, buttonPress, LOW);//reattach the button interrupt
    pinMode(encodeA, INPUT_PULLUP); //reenable the encoder interrupt
  }
  //delay(500);
}

void coolDown() {
  unsigned long waitTime;// = millis();
  unsigned long tempReadTime;
  if (digitalRead(valvePin)) {
    scrollStatus(false);
    setStatus("CLEAR");
    setStatus("VALVE-LEFT");
    while (digitalRead(valvePin)) {
      scrollStatus(true);
      delay(10);//otherwise this will fly
    }
  }
  waitTime = millis();
  digitalWrite(powerPin, HIGH);
  digitalWrite(brewPin, HIGH);
  scrollStatus(false);
  setStatus("CLEAR");
  statusMsg = "COOLING";
  while (millis() - waitTime < 10000) {
    scrollStatus(true);
    delay(10);
  }
  digitalWrite(brewPin, LOW);
  digitalWrite(powerPin, LOW);
  scrollStatus(false);
  setStatus("CLEAR");
  while (!tempFlag) {
    statusMsg = "COOLING: " + String(tempC) + char(247) + "C";
    tempSensor.requestTemperatures();
    tempReadTime = millis();
    while (millis() - tempReadTime < 1000)
    {
      scrollStatus(true);
      delay(10);
    }
    tempC = (short) tempSensor.getTempC(insideThermometer);
    if (tempC == brewTemp) {
      tempFlag = true;
    }
  }
  digitalWrite(powerPin, HIGH);
}


//set the message and status blocks
//only sets if changed unless resetStatus is true
void setDisplayType(long dialPos, bool resetStatus = false) {
  long newPosition = dialPos;
  static String oldStatus = "START";//let's keep this static to compare through calls
  String encState;//send this variable to set status
  //same as the buttonFlag checks, left side of menu is power, right side is grind, in between is brew time
  if (newPosition <= 1) {
    encState = "POWER";
    myEnc.write(1);
    newPosition = 1;
  }
  else if (newPosition > 1 && newPosition < maxTime * 2) {
    encState = "READY";
    //myEnc.write(newPosition / 2);
  }
  else {
    encState = "GRIND";
    if (newPosition > maxTime * 2 + 2) {
      encState = "GRIND";
      myEnc.write(maxTime * 2 + 3);
      newPosition = maxTime * 2 + 3;
    }

  }
  clearWindow();
  if (newPosition >= maxTime * 2) {
    setGrindScreen(newPosition);//lazy programming, but set the message screen to indicate grind 1 or 2 cups
  }
  else {
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print("Time: ");
    if (newPosition == 0) {
      display.println("0");
    }
    else {
      display.println(newPosition / 2);
    }
  }
  //Serial.println(encState);
  //Serial.println(oldStatus);
  //set the stauts block on if it's changed or explicitly requested
  if (encState != oldStatus || resetStatus) {
    setStatus(encState);
  }
  display.display();//draw the screen
  oldStatus = encState;//retain the status
}
//grind some beans
void grind(int cups) {
  /* REMOVED STEPPER MOTOR
    //could have done this bit with existing routines, but lazy
    clearWindow();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print("GRINDING");
    setStatus("BEANS");
    powerDown();//turn the machine off while brewing...takes about 5 minutes per cup
    long steps = cupTurns * cups * -1;//brew one or two cups
    stepper.enable();
    stepper.rotate(steps);//blocking, will wait here until finished
    stepper.disable();//turn off lest consume max power from wall wart
    digitalWrite(powerPin, HIGH);//turn machine back on
    //setStatus("READY");//don't need this anymore bc of setDisplayType
    inactiveTime = millis();//reset the inactive counter, surely it would have expired by now
    //clearWindow();//don't need
    //display.display();//don't need
  */
}

//let's brew some coffee
void startBrew(int brewTime) {
  int oldSetting = brewTime;
  bool steep = true;//do i want to steep the grinds?
  brewTime = steepDuration;//for how long?
  setStatus("STEEP");
  clearWindow();
  display.setTextSize(2);
  display.setCursor(0, 0);
  digitalWrite(brewPin, HIGH);//turn the pump on
  //brew for a bit to steep the grinds
  for (int i = steepDelay; i > 0; i--) {
    display.print(". ");
    display.display();
    delay(1000);
  }
  digitalWrite(brewPin, LOW);//stop brewing
  //two step for-loop, if not steeping, it will just brew
  //if steeping it will brew twice, once for steep-length, once for selection-length
  for (int i = 0; i <= 1; i++) {
    //if i'm not steep just go ahead and brew
    if (!steep) {
      digitalWrite(brewPin, HIGH);
    }
    //countdown from brewtime (steep or encoder selection) to zero
    for (brewTime; brewTime >= 0; brewTime--) {
      clearWindow();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(brewTime);
      //instead of delay, use a millis() while loop
      //allows me to scroll the status while brewing
      long thisCount = millis();
      while (millis() - thisCount <= 1000) {
        scrollStatus(true);
      }
    }
    digitalWrite(brewPin, LOW);//stop brewing
    //if I did steep, turn off the steep flag, use the encoder selection for brew time
    if (steep) {
      steep = false;
      brewTime = oldSetting;
      scrollStatus(false);
      setStatus("BREW");
    }
  }
  scrollStatus(false);
  clearWindow();
}

void scrollStatus(bool scroll) {
  static int x = 0;
  static bool scrollLeft = true;
  int textPos = ((display.width() - rectWidth - statusMsg.length() * 6) / 2) - x;
  if (scroll) {
    display.setTextSize(1);
    display.setTextWrap(false);
    if (scrollLeft) {
      if (textPos > 0) {
        x++;
      }
      else {
        scrollLeft = false;
      }
    }
    else if (!scrollLeft) {
      if (textPos <= display.width() - rectWidth - (rectThick * 2) - (statusMsg.length() * 6)) {
        x--;
      }
      else {
        scrollLeft = true;
      }
    }
    //Serial.print("X: ");
    //Serial.println(x);
    //Serial.print("textPOS: ");
    //Serial.println(textPos);
    display.fillRect(0, display.height() - 10, display.width() - rectWidth - (rectThick * 2), 10, BLACK);
    display.setCursor(textPos, display.height() - 8);
    display.println(statusMsg);
    display.display();
  }
  else {
    x = 0;
    scrollLeft = true;
  }
}

void startupScreen() {
  display.setTextSize(2); // Draw 2X-scale text
  char banner[] = "bearSpresso (v11)";
  int x    = display.width() - rectWidth - rectThick * 2;
  int minX = (-12 * strlen(banner));// + display.width() - 30; //-48;
  int level;
  while (x >= minX) {
    display.clearDisplay();
    display.setTextWrap(false);
    display.setCursor(x, 16);
    display.println(banner);
    display.display();
    x--;
  }
  display.clearDisplay();
  setScreen();
  delay(500);
  if (waterSense) {
    float pressure_inch = getPressure();
    level = waterLevel(pressure_inch);
  }
  else {
    level = 5;
  }

  for (int i = 99; i >= level; i--) {
    waterLevel((float)i, false);
    delay(2);
  }
}
void clearWindow() {
  display.fillRect(0, 0, display.width() - rectWidth - rectThick * 2, display.height() - 10 - rectThick, BLACK);

}

void setScreen() {
  display.fillRect(display.width() - rectWidth, 0, rectWidth, rectHeight, WHITE);
  display.fillRect(display.width() - rectWidth + rectThick, 0, rectWidth - rectThick * 2, rectHeight - rectThick, BLACK);
  display.drawFastHLine(rectThick, display.height() - 10 - rectThick, display.width() - rectWidth - rectThick * 2, WHITE);
  display.display();
}

void sleepTime() {
  detachInterrupt(0);
  clearWindow();
  display.setCursor((display.width() - rectWidth) / 3 - 12, 0);
  display.setTextSize(2);
  display.display();
  for (int i = 3; i > 0; i--) {
    display.print(". ");
    display.display();
    delay(1000);
  }
  powerDown();
  display.clearDisplay();
  display.drawBitmap(0, 0, bearSplash_data, display.width(), display.height(), 1);
  display.invertDisplay(true);
  //display.dim(true);
  display.ssd1306_command(0x81); //0x81
  display.ssd1306_command(0);
  display.ssd1306_command(0xD9);
  display.ssd1306_command(0);
  display.display();
  pinMode(encodeA, OUTPUT); //need to disable the encoder interrupt
  digitalWrite(encodeA, LOW); //ditto^
  attachInterrupt(0, wakeUp, LOW);
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  powerOn();
}
//turn off the espresso machine
void powerDown() {
  powerOnFlag = false;
  digitalWrite(powerPin, LOW);
}

//turn the relay on for the espresso machine power
void powerOn() {
  //  tempSensor.requestTemperatures();
  //  tempC = (short) tempSensor.getTempC(insideThermometer);
  powerOnFlag = true;
  digitalWrite(powerPin, HIGH);
  //clearWindow();
  //display.dim(false);
  display.ssd1306_command(0x81); //0x81
  display.ssd1306_command(160);
  display.ssd1306_command(0xD9);
  display.ssd1306_command(34);
  display.display();
  delay(3000);
  display.invertDisplay(false);
  display.clearDisplay();
  setScreen();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("Time: 1");
  //waterLevel(50.0);
  if (waterSense) {
    waterLevel(getPressure());
  }
  setStatus("READY");
  myEnc.write(3);
  display.display();
  attachInterrupt(0, buttonPress, LOW);
  pinMode(encodeA, INPUT_PULLUP); //reenable the encoder interrupt
  buttonFlag = false;
  inactiveTime = millis();//reset the sleep timer
  //pinMode(encodeA,INPUT_PULLUP);
  //Encoder myEnc(3, 4);
}

//wake up the device from an external interrupt (the button press)
void wakeUp() {
  detachInterrupt(0);
  //  pinMode(encodeA, INPUT_PULLUP); //reenable the encoder interrupt
  sleep_disable();//AVR function
}


void buttonPress() {
  //software debounce
  static unsigned long last_time = 0;
  unsigned long this_time = millis();
  if (this_time - last_time > 50) {
    buttonFlag = true;
  }
  last_time = this_time;
}

void setStatus(String encState) {
  //String statusMsg;
  if (encState == "POWER") {
    statusMsg = "PUSH TO TURNOFF";
  }
  else if (encState == "HEATING") {
    String temp = String(tempC);
    statusMsg = encState + temp;
  }
  else if (encState == "READY") {
    statusMsg = "PUSH TO BREW";
  }
  else if (encState == "START") {
    statusMsg = "PUSH TO WAKE";
  }
  else if (encState == "OFF") {
    statusMsg = "POWERING OFF";
  }
  else if (encState == "WATER") {
    statusMsg = "ADD WATER";
  }
  else if (encState == "VALVE-LEFT") {
    statusMsg = String("TURN VALVE ") + char(27);
  }
  else if (encState == "VALVE-RIGHT") {
    statusMsg = String("TURN VALVE ") + char(26);
  }
  else if (encState == "CLEAR") {
    statusMsg = " ";
  }
  else if (encState == "BREW") {
    statusMsg = "BREWING!";
  }
  else if (encState == "COMPLETE") {
    statusMsg = "COFFEE IS READY!";
  }
  else if (encState == "STEEP") {
    statusMsg = "STEEPING";
  }
  else if (encState == "GRIND") {
    statusMsg = "PUSH TO GRIND";
  }
  else if (encState == "BEANS") {
    statusMsg = "PLEASE WAIT...";
  }
  display.fillRect(0, display.height() - 10, display.width() - rectWidth - rectThick * 2, 10, BLACK);
  display.setTextSize(1);
  display.setCursor((display.width() - rectWidth - statusMsg.length() * 6) / 2, display.height() - 8);
  display.println(statusMsg);
  display.display();
}

void setGrindScreen(int reading) {
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("CUPS: ");
  if (reading >= maxTime * 2 && reading <= maxTime * 2 + 2) {
    display.print("1");
  }
  else {
    display.print("2");
  }

}
int waterLevel(float reading = 20.0, bool readLevel = true) {
  display.setTextSize(1);
  int charOffset = 8; //pixel height for characters in text size 1
  float percent;
  String str;
  int percentInt;
  if (readLevel == true) {
    percent = map(reading * 10, minWaterLevel * 10, maxWaterLevel * 10, 0.0, 99.0);
    percent = constrain(percent, 0, 99);
    str = String(percent);
    //Serial.println(percent);
  }
  else {
    percent = (int)reading;
  }
  percentInt = (int)percent;
  str = String(percentInt);
  int level = map((int)percent, 0, 99, rectHeight - rectThick, 0);
  str.concat("%");
  char levelNum[4];
  str.toCharArray(levelNum, 4);
  //clear the barrel
  display.fillRect(display.width() - rectWidth + rectThick, 0, rectWidth - rectThick * 2, rectHeight - rectThick, BLACK);
  //fill the barrel to the approproiate height
  display.fillRect(display.width() - rectWidth, level , rectWidth, rectHeight - rectThick - level, WHITE);
  //clear the string background
  display.fillRect(display.width() - rectWidth + rectThick + 1, 0, rectWidth - 2 * rectThick - 2, charOffset, BLACK);
  display.setCursor(display.width() - rectWidth / 2 - (strlen(levelNum)) * 6 / 2, 0);
  display.print(levelNum);
  display.display();
  return percentInt;
}

float getPressure() {
  //static float maxDiff = 0;
  sensors_event_t atmosEvent;
  bmp.getEvent(&atmosEvent);
  //float pressure_hpa = mpr.readPressure() + 3.3;
  //float pressure_inches = ((pressure_hpa - atmosEvent.pressure) / 68.947572932) * psiToInch;
  //pressure_inches = (pressure_inches <= 0) ? 0 : pressure_inches;
  //return pressure_inches;
  return atmosEvent.pressure / 68.947572932;
}

void printAddress(DeviceAddress deviceAddress)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  delay(1000);
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) display.print("0");
    display.print(deviceAddress[i], HEX);
  }
  display.display();
}
