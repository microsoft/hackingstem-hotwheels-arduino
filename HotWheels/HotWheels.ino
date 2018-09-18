/*
Tuned Mass Dampner (TMD) code using Arduino UNO and Adafruit LIS3DH breakout board
More detials on TMD: https://www.microsoft.com/en-us/education/education-workshop/seismograph.aspx
More details on LIS3DH: https://www.adafruit.com/product/2809

This code captures accelerometer data over i2c, but not relying on any external libraries, and sends it over serial.

Adi Azulay, 2017 Microsoft EDU Workshop
*/

#include <Wire.h>


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

/*
 *  Digital Pins for Gate Reading
 */
const byte gate1pin = 2;
const byte gate2pin = 3;
const byte gate3pin = 4;
const byte gate4pin = 5;
const byte gate5pin = 6;
const byte gate6pin = 7;
const byte gate7pin = 8;
const byte gate8pin = 9;
const byte gate9pin = 10;

/*
*   Game Control Variables
*/

  bool gate1state = 0;
  unsigned long gate1time = 0;
  bool gate1done=0;

  bool gate2state = 0;
  unsigned long gate2time = 0;
  bool gate2done=0;

  bool gate3state = 0;
  unsigned long gate3time = 0;
  bool gate3done=0;

  bool gate4state = 0;
  unsigned long gate4time = 0;
  bool gate4done=0;

  bool gate5state = 0;
  unsigned long gate5time = 0;
  bool gate5done=0;

  bool gate6state = 0;
  unsigned long gate6time = 0;
  bool gate6done=0;

  bool gate7state = 0;
  unsigned long gate7time = 0;
  bool gate7done=0;

  bool gate8state = 0;
  unsigned long gate8time = 0;
  bool gate8done=0;

  bool gate9state = 0;
  unsigned long gate9time = 0;
  bool gate9done=0;

  bool force1done = 0;

  int prevValue = 0;
  int value = 0;

  float force1time = 0;

/*
 * Program control variables
*/
unsigned long startTime = 0;
byte raceFinished = 0;

/*
 * Excel program control variables
 */
int resetTrial = 0;                       //resets all variables
int numberOfGates = 9;                    //sets the final gate variable (overridden by Excel)

/*
 * Serial data variables
 */
const String mDELIMETER = ",";            // Cordoba expects a comma delimeted string of data
String mInputString = "";                 // string variable to hold incoming data
boolean mStringComplete = false;          // variable to indicate the string is complete (newline found)
int mSerial_Interval = 50;                // Intervel between serial writes
unsigned long mSerial_PreviousTime;       // Timestamp to track interval

const int dataRate = 10;      // Change this variable to increase the frequency that data is sent to Excel

void setup(){
                                            //Initilize LIS3DH and set to +/- 16g Scale Factor
  Wire.begin();

  Wire.beginTransmission(LIS3DH_ADDRESS);   //Connects to LIS3DH via i2c
  Wire.write (LIS3DH_REG_WHOAMI);            //Check that board is connected
  Wire.endTransmission(true);
  Wire.requestFrom (LIS3DH_ADDRESS, 1);
  uint8_t deviceID = Wire.read();

  while (deviceID != 0x33){
    delay(1);
  }

  writeRegister8 (LIS3DH_REG_CTRL1, 0x07);      //Turn on all axes and set to normal mode

  //set data rate
  uint8_t accelDataRate = readRegister8 (LIS3DH_REG_CTRL1);
  accelDataRate &= ~(0xF0);
  accelDataRate |= 0b0111 << 4;                //Change variable to write
  writeRegister8 (LIS3DH_REG_CTRL1, accelDataRate);      //Set data rate to 400 mHz, used to manage power consuption
  writeRegister8 (LIS3DH_REG_CTRL4, 0x88);      //Enebles High Res and BDU
  writeRegister8 (LIS3DH_REG_CTRL3, 0x10);       // DRDY on INT1
  writeRegister8 (LIS3DH_REG_TEMPCFG, 0x80);      //Activate ADC outputs

  //Set read scale
  uint8_t rangeControl = readRegister8 (LIS3DH_REG_CTRL4);
  rangeControl &= ~(0x30);
  rangeControl |= LIS3DH_RANGE_8_G << 4;                //Change variable to write make sure to also update the scale factor
  writeRegister8 (LIS3DH_REG_CTRL4, rangeControl);


  Serial.begin (9600);

  pinMode(gate1pin, INPUT);
  pinMode(gate2pin, INPUT);
  pinMode(gate3pin, INPUT);
  pinMode(gate4pin, INPUT);
  pinMode(gate5pin, INPUT);
  pinMode(gate6pin, INPUT);
  pinMode(gate7pin, INPUT);
  pinMode(gate8pin, INPUT);
  pinMode(gate9pin, INPUT);

  resetTrialVariables();
}

void loop(){
  processIncomingSerial();
  if(resetTrial==1){
    resetTrialVariables();
  }
  processDigitalSensors();
  processSwitches();
  processForceSensor();
  processOutgoingSerial();
}

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
void processSwitches(){
    // Digitial switch reads
    if(raceFinished==0){
      if(gate1state==HIGH && gate1done==0){
          gate1time = 0;
          gate1done=1;
          startTime = millis();
      }

      if(gate2state==HIGH && gate2done==0){
          gate2time = millis() - startTime;
          gate2done=1;
          if(numberOfGates==2){
            raceFinished = 1;
          }
      }

      if(gate3state==HIGH && gate3done==0){
          gate3time = millis() - startTime;
          gate3done=1;
          if(numberOfGates==3){
            raceFinished = 1;
          }
      }

      if(gate4state==HIGH && gate4done==0){
          gate4time = millis() - startTime;
          gate4done=1;
          if(numberOfGates==4){
            raceFinished = 1;
          }
      }

      if(gate5state==HIGH && gate5done==0){
          gate5time = millis() - startTime;
          gate5done=1;
          if(numberOfGates==5){
            raceFinished = 1;
          }
      }

      if(gate6state==HIGH && gate6done==0){
          gate6time = millis() - startTime;
          gate6done=1;
          if(numberOfGates==6){
            raceFinished = 1;
          }
      }

      if(gate7state==HIGH && gate7done==0){
          gate7time = millis() - startTime;
          gate7done=1;
          if(numberOfGates==7){
            raceFinished = 1;
          }
      }

      if(gate8state==HIGH && gate8done==0){
          gate8time = millis() - startTime;
          gate8done=1;
          if(numberOfGates==8){
            raceFinished = 1;
          }
      }

      if(gate9state==HIGH && gate9done==0){
          gate9time = millis() - startTime;
          gate9done=1;
          if(numberOfGates==9){
            raceFinished = 1;
          }
      }
    }
}

void processForceSensor(){
  float maxYAxis = 0;
  if(raceFinished==1 && force1done==0)
  {
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
    force1time = maxYAxis;            //get the time interval passed during impact (for Excel)
    force1done = 1;
  }
}

void resetTrialVariables(){
  raceFinished = 0;
  force1done = 0;

  gate1time = 999;
  gate1done=0;

  gate2time = 0;
  gate2done=0;

  gate3time = 0;
  gate3done=0;

  gate4time = 0;
  gate4done=0;

  gate5time = 0;
  gate5done=0;

  gate6time = 0;
  gate6done=0;

  gate7time = 0;
  gate7done=0;

  gate8time = 0;
  gate8done=0;

  gate9time = 0;
  gate9done=0;

  force1time = 0;

  resetTrial = 0;
}

/*
 * -------------------------------------------------------------------------------------------------------
 * INCOMING SERIAL DATA FROM EXCEL PROCESSING CODE--------------------------------------------------------
 * -------------------------------------------------------------------------------------------------------
 */

void processIncomingSerial()
{
  getSerialData();
  parseSerialData();
}

/*
 * getSerialData()
 *
 * Gathers bits from serial port to build mInputString
 */
void getSerialData()
{
  while (Serial.available()) {
    char inChar = (char)Serial.read();      // get new byte
    mInputString += inChar;                 // add it to input string
    if (inChar == '\n') {                   // if we get a newline...
      mStringComplete = true;               // we have a complete string of data to process
    }
  }
}

/*
 * parseSerialData()
 *
 * Parse all program control variables and data from Excel
 */
void parseSerialData()
{
  if (mStringComplete) { // process data from mInputString to set program variables.
    //set variables using: var = getValue(mInputString, ',', index).toInt(); // see getValue function below

    resetTrial    = getValue(mInputString, ',', 0).toInt(); // Data Out column A5

    String data = getValue(mInputString, ',', 1);           // Data Out column B5
    if(data != ""){
      numberOfGates = data.toInt(); //getValue(mInputString, ',', 1); //.toInt(); // Data Out column B5
    }

    mInputString = "";                         // reset mInputString
    mStringComplete = false;                   // reset stringComplete flag
  }
}

/*
 * getValue()
 *
 * Gets value from mInputString using an index. Each comma delimited value in mInputString
 * is counted and the value of the matching index is returned.
 */
String getValue(String mDataString, char separator, int index)
{
  // mDataString is mInputString, separator is a comma, index is where we want to look in the data 'array'
  int matchingIndex = 0;
  int strIndex[] = {0, -1};
  int maxIndex = mDataString.length()-1;
  for(int i=0; i<=maxIndex && matchingIndex<=index; i++){     // loop until end of array or until we find a match
    if(mDataString.charAt(i)==separator || i==maxIndex){      // if we hit a comma OR we are at the end of the array
      matchingIndex++;                                        // increment matchingIndex to keep track of where we have looked
      strIndex[0] = strIndex[1]+1;                            // increment first substring index
      // ternary operator in objective c - [condition] ? [true expression] : [false expression]
      strIndex[1] = (i == maxIndex) ? i+1 : i;                // set second substring index
    }
  }
  return matchingIndex>index ? mDataString.substring(strIndex[0], strIndex[1]) : ""; // if match return substring or ""
}

/*
 * -------------------------------------------------------------------------------------------------------
 * OUTGOING SERIAL DATA TO EXCEL PROCESSING CODE----------------------------------------------------------
 * -------------------------------------------------------------------------------------------------------
 */
void processOutgoingSerial()
{
  if((millis() - mSerial_PreviousTime) > mSerial_Interval) // Write at intervals to prevent bogging down script
  {
    mSerial_PreviousTime = millis(); // Reset interval timestamp
    sendDataToSerial();
  }
}

void sendDataToSerial()
{
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

    //force time in microseconds
    Serial.print(force1time);
    Serial.print(mDELIMETER);

    Serial.println();           // Line ending character
}

void writeRegister8 (uint8_t reg, uint8_t val){
  Wire.beginTransmission (LIS3DH_ADDRESS);
  Wire.write (reg);
  Wire.write (val);
  Wire.endTransmission();
}

uint8_t readRegister8 (uint8_t reg){
  Wire.beginTransmission (LIS3DH_ADDRESS);
  Wire.write (reg);
  Wire.endTransmission();

  Wire.requestFrom(LIS3DH_ADDRESS, 1);
  uint8_t val = Wire.read();
  return val;
  Wire.endTransmission();

}
