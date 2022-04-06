//===-------__ Hacking STEM – HotWheels_Accel.X.X.X.ino – Arduino __-------===//
// For use with the Measuring Speed to Understand Forces & Motion lesson plan
// available from Microsoft Education Workshop at 
// https://education.microsoft.com/lesson/55389d0b
// http://aka.ms/hackingSTEM
//
// Overview:
// This project times the interval in milliseconds as cars pass between
// switch gates and then measures the gforce of the car impacting the
// accelerometer at the end stop.
//
// This may be implemented with 2 to 9 switch gates. The number of switch
// gates is configured by Excel via Serial connection.
//
// This project uses an Adafruit LIS3DH breakout board, information at:
// https://www.adafruit.com/product/2809
// This project uses an Arduino UNO microcontroller board, information at:
// https://www.arduino.cc/en/main/arduinoBoardUno
//
// Comments, contributions, suggestions, bug reports, and feature requests
// are welcome! For source code and bug reports see:
// https://github.com/microsoft/hackingstem-hotwheels-arduino
//
// Copyright 2018, Adi Azulay Microsoft EDU Workshop - HackingSTEM
// MIT License terms detailed in LICENSE.txt
//===----------------------------------------------------------------------===//

// Wire is a library that allows communication via i2c with devices such as
// the LIS3DH accelerometer. See https://www.arduino.cc/en/Reference/Wire
#include <Wire.h>

// Declaring constant variables to allow us to refer to pins by name, this
// makes code easier to understand. Here are all the gate pins.
const byte gate1pin = 2;
const byte gate2pin = 3;
const byte gate3pin = 4;
const byte gate4pin = 5;
const byte gate5pin = 6;
const byte gate6pin = 7;
const byte gate7pin = 8;
const byte gate8pin = 9;
const byte gate9pin = 10;

// To time each gate we declare variable to allow us to track when the
// gate is triggered.
bool gate1state = LOW;       //State of gate at last read
unsigned long gate1time = 0; //Time elapsed between start and last read
bool gate1done = false;      //If gate was processed during current race

bool gate2state = LOW;       //State of gate at last read
unsigned long gate2time = 0; //Time elapsed between start and last read
bool gate2done = false;      //If gate was processed during current race

bool gate3state = LOW;       //State of gate at last read
unsigned long gate3time = 0; //Time elapsed between start and last read
bool gate3done = false;      //If gate was processed during current race

bool gate4state = LOW;       //State of gate at last read
unsigned long gate4time = 0; //Time elapsed between start and last read
bool gate4done = false;      //If gate was processed during current race

bool gate5state = LOW;       //State of gate at last read
unsigned long gate5time = 0; //Time elapsed between start and last read
bool gate5done = false;      //If gate was processed during current race

bool gate6state = LOW;       //State of gate at last read
unsigned long gate6time = 0; //Time elapsed between start and last read
bool gate6done = false;      //If gate was processed during current race

bool gate7state = LOW;       //State of gate at last read
unsigned long gate7time = 0; //Time elapsed between start and last read
bool gate7done = false;      //If gate was processed during current race

bool gate8state = LOW;       //State of gate at last read
unsigned long gate8time = 0; //Time elapsed between start and last read
bool gate8done = false;      //If gate was processed during current race

bool gate9state = LOW;       //State of gate at last read
unsigned long gate9time = 0; //Time elapsed between start and last read
bool gate9done = false;      //If gate was processed during current race


// Race variables
unsigned long startTime = 0; // Millis timestamp of race start
bool raceFinished = false;   // If final gate is completed
bool forceReadDone = false;     // If force sensor is read
float gforce = 0;        // Accelerometer force output

// Config & control variables, set via Serial (by Excel)
// to configure and trigger race
int resetTrial = 0;    //resets all variables
int numberOfGates = 3; //sets the final gate variable (overridden by Excel)

// Serial data variables, used when sending and receiving data
const String mDELIMETER = ",";   // comma delimited values
String mInputString = "";        // incoming data
boolean mStringComplete = false; // if incoming line is complete (newline found)
int mSerial_Interval = 50;       // interval between serial writes
unsigned long mSerial_PreviousTime; // timestamp to track interval
const int dataRate = 10;         // frequency of serial writes, in milliseconds


// setup() method is a special Arduino method that runs once at beginning of
// program. Typically used to initialize pins and connections
void setup(){
  //initialize serial connection (to communicate with excel)
  Serial.begin (9600);

  // initialized communication with accelerometer
  // note, if accelerometer has problems, program can hange here
  initializeAccelerometer();

  // Set each of our gate pins to be used as input pins
  pinMode(gate1pin, INPUT);
  pinMode(gate2pin, INPUT);
  pinMode(gate3pin, INPUT);
  pinMode(gate4pin, INPUT);
  pinMode(gate5pin, INPUT);
  pinMode(gate6pin, INPUT);
  pinMode(gate7pin, INPUT);
  pinMode(gate8pin, INPUT);
  pinMode(gate9pin, INPUT);

  // Reset all race variable, so we're ready to go!
  resetTrialVariables();
}



// loop() method is a special Arduino function that runs over and over. This
// function executes from top to bottom and then starts again, it runs forever!
void loop(){
  processIncomingSerial(); // See if any commands have come in (from Excel)
  if(resetTrial==1){       // If a race was already run, clear old data
    resetTrialVariables();
  }
  processDigitalSensors(); // Read all gate switches
  processSwitches();       // Process any changes to switch state
  processForceSensor();    // Read force sensor
  processOutgoingSerial(); // Write any results to Serial (to Excel)
}


// Quickly read state of each gate switch pin and write it to variables
void processDigitalSensors(){
  gate1state = digitalRead(gate1pin);
  gate2state = digitalRead(gate2pin);
  gate3state = digitalRead(gate3pin);
  gate4state = digitalRead(gate4pin);
  gate5state = digitalRead(gate5pin);
  gate6state = digitalRead(gate6pin);
  gate7state = digitalRead(gate7pin);
  gate8state = digitalRead(gate8pin);
  gate9state = digitalRead(gate9pin);
}


// Examine gate switch pine state and update timing if triggered
void processSwitches(){
    if(raceFinished==false){
      // If first gate is triggered, we set startTime to current
      // timestamp, in milliseconds.
      if(gate1state==HIGH && gate1done==false){
          gate1time = 0;
          gate1done=true;
          startTime = millis();
      }

      // When gate is triggered we calculate the elapsed time since
      // the first gate was triggered
      if(gate2state==HIGH && gate2done==false){
          gate2time = millis() - startTime;
          gate2done=true;
          if(numberOfGates==2){ // If this is last gate, race as finished
            raceFinished = true;
          }
      }

      // When gate is triggered we calculate the elapsed time since
      // the first gate was triggered
      if(gate3state==HIGH && gate3done==false){
          gate3time = millis() - startTime;
          gate3done=true;
          if(numberOfGates==3){ // If this is last gate, race as finished
            raceFinished = true;
          }
      }

      // When gate is triggered we calculate the elapsed time since
      // the first gate was triggered
      if(gate4state==HIGH && gate4done==false){
          gate4time = millis() - startTime;
          gate4done=true;
          if(numberOfGates==4){ // If this is last gate, race as finished
            raceFinished = true;
          }
      }

      // When gate is triggered we calculate the elapsed time since
      // the first gate was triggered
      if(gate5state==HIGH && gate5done==false){
          gate5time = millis() - startTime;
          gate5done=true;
          if(numberOfGates==5){ // If this is last gate, race as finished
            raceFinished = true;
          }
      }

      // When gate is triggered we calculate the elapsed time since
      // the first gate was triggered
      if(gate6state==HIGH && gate6done==false){
          gate6time = millis() - startTime;
          gate6done=true;
          if(numberOfGates==6){ // If this is last gate, race as finished
            raceFinished = true;
          }
      }

      // When gate is triggered we calculate the elapsed time since
      // the first gate was triggered
      if(gate7state==HIGH && gate7done==false){
          gate7time = millis() - startTime;
          gate7done=true;
          if(numberOfGates==7){ // If this is last gate, race as finished
            raceFinished = true;
          }
      }

      // When gate is triggered we calculate the elapsed time since
      // the first gate was triggered
      if(gate8state==HIGH && gate8done==false){
          gate8time = millis() - startTime;
          gate8done=true;
          if(numberOfGates==8){ // If this is last gate, race as finished
            raceFinished = true;
          }
      }

      // When gate is triggered we calculate the elapsed time since
      // the first gate was triggered
      if(gate9state==HIGH && gate9done==false){
          gate9time = millis() - startTime;
          gate9done=true;
          if(numberOfGates==9){ // If this is last gate, race as finished
            raceFinished = true;
          }
      }
    }
}


// If race is finished poll force sensor for a reading
void processForceSensor(){
  if(raceFinished==true && forceReadDone==false) {
    gforce = getMaxYAxisReadFromForceSensor(); // get force reading
    forceReadDone = true;
  }
}

// Resets all variables so we're ready to run another race trail
void resetTrialVariables(){
  raceFinished = false;
  forceReadDone = false;

  gate1time = 999;
  gate1done = false;

  gate2time = 0;
  gate2done = false;

  gate3time = 0;
  gate3done = false;

  gate4time = 0;
  gate4done = false;

  gate5time = 0;
  gate5done = false;

  gate6time = 0;
  gate6done = false;

  gate7time = 0;
  gate7done = false;

  gate8time = 0;
  gate8done = false;

  gate9time = 0;
  gate9done = false;

  gforce = 0;

  resetTrial = 0;
}

// Read any lines of text, if available and parse commands and configuration
void processIncomingSerial() {
  getSerialData();
  parseSerialData();
}

// Writes data to Serial (for excel) at a maximum rate of mSerial_Interval
void processOutgoingSerial() {
  // Write at intervals to prevent bogging down script
  if((millis() - mSerial_PreviousTime) > mSerial_Interval)
  {
    mSerial_PreviousTime = millis(); // Reset interval timestamp
    sendDataToSerial();
  }
}


//===----------------------------------------------------------------------===//
//  Note: Code below this line may be technical and somewhat confusing, the
//  purpose of these functions are to do some complicated things such as
//  interfacing with external devices.
//
//  To communicate with LIS3DH without requiring external libraries, we had
//  to implement some code in this section may seem very cryptic! Don't worry
//  it's hard for us to understand as well :)
//===----------------------------------------------------------------------===//


// Looks Serial data and read it character by character into a string
void getSerialData() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); // get new byte
    mInputString += inChar;            // add it to input string
    if (inChar == '\n') {              // if we get a newline...
      mStringComplete = true; // we have a complete string of data to process
    }
  }
}


// Take input string and parse commands and configuration
void parseSerialData() {
  if (mStringComplete) { // process data from mInputString to set variables.
    resetTrial = getValue(mInputString, ',', 0).toInt(); // Data Out column A5
    String data = getValue(mInputString, ',', 1);        // Data Out column B5
    if(data != ""){
      numberOfGates = data.toInt();  // convert to integer, if not empty
    }

    mInputString = "";        // reset mInputString
    mStringComplete = false;  // reset stringComplete flag
  }
}


// Gets value from mDataString using an index and separator
// Example:
// given 'alice,bob,dana' and separator ','
// index 0 returns 'alice'
// index 1 returns 'bob'
// index 2 returns 'dana'
//
// mDataString: String as read from Serial (mInputString)
// separator: Character used to separate values (a comma)
// index: where we want to look in the data 'array' for value
String getValue(String mDataString, char separator, int index) {
  int matchingIndex = 0;
  int strIndex[] = {0, -1};
  int maxIndex = mDataString.length()-1;
  // loop until end of array or until we find a match
  for(int i=0; i<=maxIndex && matchingIndex<=index; i++){
    if(mDataString.charAt(i)==separator || i==maxIndex){ // if we hit a comma
                                                         // OR end of the array
      matchingIndex++;   // increment to track where we have looked
      strIndex[0] = strIndex[1]+1;   // increment first substring index
      strIndex[1] = (i == maxIndex) ? i+1 : i;   // set second substring index
    }
  }
  // if match return substring or ""
  if (matchingIndex > index) {
    return mDataString.substring(strIndex[0], strIndex[1]);
  } else {
    return "";
  }
}

// Constructs comma delimited list of gate times and force, then writes it
// to serial (for Excel).
// Writes 9 gate time stamps and one force
// example 11225,13535,18929,0,0,0,0,0,2.3223
void sendDataToSerial() {
    //gate times in milliseconds
    Serial.print(gate1time);
    Serial.print(mDELIMETER);

    Serial.print(gate2time);
    Serial.print(mDELIMETER);

    Serial.print(gate3time);
    Serial.print(mDELIMETER);

    Serial.print(gate4time);
    Serial.print(mDELIMETER);

    Serial.print(gate5time);
    Serial.print(mDELIMETER);

    Serial.print(gate6time);
    Serial.print(mDELIMETER);

    Serial.print(gate7time);
    Serial.print(mDELIMETER);

    Serial.print(gate8time);
    Serial.print(mDELIMETER);

    Serial.print(gate9time);
    Serial.print(mDELIMETER);

    Serial.print(gforce);
    Serial.print(mDELIMETER);

    Serial.println();           // Line ending character
}

// List of registers used by accelerometer
#define LIS3DH_ADDRESS           0x18
#define LIS3DH_REG_STATUS1       0x07
#define LIS3DH_REG_WHOAMI        0x0F
#define LIS3DH_REG_TEMPCFG       0x1F
#define LIS3DH_REG_CTRL1         0x20
#define LIS3DH_REG_CTRL3         0x22
#define LIS3DH_REG_CTRL4         0x23
#define LIS3DH_REG_OUT_Y_L       0x2A
#define LIS3DH_REG_OUT_Y_H       0x2B
#define LIS3DH_8G_SCALE_FACTOR  .00024414f
#define LIS3DH_RANGE_8_G         0b10   // +/- 8g


// Initilize LIS3DH see LIS3DH spec sheet
void initializeAccelerometer() {
  Wire.begin();
  Wire.beginTransmission(LIS3DH_ADDRESS); //Connects to LIS3DH via i2c
  Wire.write (LIS3DH_REG_WHOAMI);         //Check that board is connected
  Wire.endTransmission(true);
  Wire.requestFrom (LIS3DH_ADDRESS, 1);
  uint8_t deviceID = Wire.read();

  while (deviceID != 0x33){
    for (int time = 0; time < 10000; time++){
      delay(1); // TODO review looks like a bug... hangs forever if no LIS3DH on first Wire.read() should at least write "No LISD3H" to serial while it hangs
    }
    if (deviceID == 0x33){
      break;
    }
    Serial.println("Accelerometer Not Found");
    delay (20000);
    break;
  }

  //Turn on all axes and set to normal mode
  writeRegister8 (LIS3DH_REG_CTRL1, 0x07);

  //set data rate
  uint8_t accelDataRate = readRegister8 (LIS3DH_REG_CTRL1);
  accelDataRate &= ~(0xF0);
  accelDataRate |= 0b0111 << 4;   //Change variable to write

  //Set data rate to 400 mHz, used to manage power consumption
  writeRegister8 (LIS3DH_REG_CTRL1, accelDataRate);
  writeRegister8 (LIS3DH_REG_CTRL4, 0x88);     //Enebles High Res and BDU
  writeRegister8 (LIS3DH_REG_CTRL3, 0x10);     // DRDY on INT1
  writeRegister8 (LIS3DH_REG_TEMPCFG, 0x80);   //Activate ADC outputs

  //Set read scale
  uint8_t rangeControl = readRegister8 (LIS3DH_REG_CTRL4);
  rangeControl &= ~(0x30);
  //Change variable to write make sure to also update the scale factor
  rangeControl |= LIS3DH_RANGE_8_G << 4;
  writeRegister8 (LIS3DH_REG_CTRL4, rangeControl);
}

// Reads Y axis force values from accelerometer and returns
// the highest of 100 sequential reads over a 100ms timespan
float getMaxYAxisReadFromForceSensor() {
    float maxYAxis = 0;
    for (int i = 0; i <= 100; i++){
      Wire.beginTransmission(LIS3DH_ADDRESS);
      Wire.write(LIS3DH_REG_OUT_Y_L | 0x80);
      Wire.endTransmission();

      Wire.requestFrom(LIS3DH_ADDRESS, 2);
      while (Wire.available() < 2);

      uint8_t yla = Wire.read();
      uint8_t yha = Wire.read();

      float yAxis = yha << 8 | yla;

      yAxis = yAxis * LIS3DH_8G_SCALE_FACTOR;
      if (yAxis > maxYAxis) {maxYAxis = yAxis;}

      delay (1);
    }
    return maxYAxis;
}

// configuration of accelerometer, LIS3DH datasheet
void writeRegister8 (uint8_t reg, uint8_t val){
  Wire.beginTransmission (LIS3DH_ADDRESS);
  Wire.write (reg);
  Wire.write (val);
  Wire.endTransmission();
}

// configuration of accelerometer, LIS3DH datasheet
uint8_t readRegister8 (uint8_t reg){
  Wire.beginTransmission (LIS3DH_ADDRESS);
  Wire.write (reg);
  Wire.endTransmission();

  Wire.requestFrom(LIS3DH_ADDRESS, 1);
  uint8_t val = Wire.read();
  return val;
  Wire.endTransmission();
}
