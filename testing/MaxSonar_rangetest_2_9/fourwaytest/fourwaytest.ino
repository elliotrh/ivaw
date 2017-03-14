#include <Adafruit_INA219.h>
#include "Wire.h"


#define address 0x1E //0011110b, I2C 7bit address of HMC5883


//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8-bit 0xE0
#define SensorAddress byte(0x70)
//The sensors ranging command has a value of 0x51
#define RangeCommand byte(0x51)
//These are the two commands that need to be sent in sequence to change the sensor address
#define ChangeAddressCommand1 byte(0xAA)
#define ChangeAddressCommand2 byte(0xA5)

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

// two PWM pins
const int pingPinL = 7;
const int pingPinR = 6;

//six Motor Pins
const int mPin_Right_UU = 34;
const int mPin_Right_UM = 35;
const int mPin_Right_LM = 32;
const int mPin_Right_LL = 33;
const int mPin_Center_UU = 30;
const int mPin_Center_UM = 31;
const int mPin_Center_LM = 40;
const int mPin_Center_LL = 41;
const int mPin_Left_UU = 38;
const int mPin_Left_UM = 39;
const int mPin_Left_LM = 36;
const int mPin_Left_LL = 37;

unsigned long prevTime = 0;
unsigned long currTime = 0;

void setup() {
 Serial.begin(115200); //Open serial connection at 9600? baud
 Wire.begin(); //Initiate Wire library for I2C communications with the I2CXL-MaxSonar-EZ

/*  #ifndef ESP8266
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
  uint32_t currentFrequency;
    */
  //Serial.println("Hello!");
  pinMode(mPin_Right_UU, OUTPUT);
  pinMode(mPin_Right_UM, OUTPUT);
  pinMode(mPin_Right_LM, OUTPUT);
  pinMode(mPin_Right_LL, OUTPUT);
  pinMode(mPin_Center_UU, OUTPUT);
  pinMode(mPin_Center_UM, OUTPUT);
  pinMode(mPin_Center_LM, OUTPUT);
  pinMode(mPin_Center_LL, OUTPUT);
  pinMode(mPin_Left_UU, OUTPUT);
  pinMode(mPin_Left_UM, OUTPUT);
  pinMode(mPin_Left_LM, OUTPUT);
  pinMode(mPin_Left_LL, OUTPUT);

  //Startup Sequence to test all motors at startup
  turnOffAll();
    delay(500);
    digitalWrite(mPin_Right_UU, LOW);
    delay(500);
    digitalWrite(mPin_Right_UU, HIGH);
    delay(500);

    digitalWrite(mPin_Right_UM, LOW);
    delay(500);
    digitalWrite(mPin_Right_UM, HIGH);
    delay(500);

    digitalWrite(mPin_Right_LM, LOW);
    delay(500);
    digitalWrite(mPin_Right_LM, HIGH);
    delay(500);

    digitalWrite(mPin_Right_LL, LOW);
    delay(500);
    digitalWrite(mPin_Right_LL, HIGH);
    delay(500);

    digitalWrite(mPin_Left_UU, LOW);
    delay(500);
    digitalWrite(mPin_Left_UU, HIGH);
    delay(500);

    digitalWrite(mPin_Left_UM, LOW);
    delay(500);
    digitalWrite(mPin_Left_UM, HIGH);
    delay(500);

    digitalWrite(mPin_Left_LM, LOW);
    delay(500);
    digitalWrite(mPin_Left_LM, HIGH);
    delay(500);

    digitalWrite(mPin_Left_LL, LOW);
    delay(500);
    digitalWrite(mPin_Left_LL, HIGH);
    delay(500);

    digitalWrite(mPin_Center_UU, LOW);
    delay(500);
    digitalWrite(mPin_Center_UU, HIGH);
    delay(500);

    digitalWrite(mPin_Center_UM, LOW);
    delay(500);
    digitalWrite(mPin_Center_UM, HIGH);
    delay(500);

    digitalWrite(mPin_Center_LM, LOW);
    delay(500);
    digitalWrite(mPin_Center_LM, HIGH);
    delay(500);

    digitalWrite(mPin_Center_LL, LOW);
    delay(500);
    digitalWrite(mPin_Center_LL, HIGH);
    delay(500);
 
}

void loop(){
//turnOffAll();

  // PARALLAX PING SECTION
  // data vars
  long durationL, inchesL, cmL, durationR, inchesR, cmR, threshold, avgL, avgR, T, d, stopdist;
  int count = 0;
  //threshold = 10; //10cm ~ 4in
  //d = 10;

  //d == 6in ~ 15cm
  d = 25;

  avgL = 0;
  avgR = 0;
  
  while(count < 100){
  avgL += durationOfPing(pingPinL);
  avgR += durationOfPing(pingPinR);
  count++;
  if(count == 25) turnOffAll();
  }

  durationL = avgL/count;
  durationR = avgR/count;

  // convert the time into a distance
  inchesL = microsecondsToInches(durationL);
  inchesR = microsecondsToInches(durationR);
  cmL = microsecondsToCentimeters(durationL);
  cmR = microsecondsToCentimeters(durationR);

  threshold = cmR*cmR - cmL*cmL;
  T = d*d;
  stopdist = 100;



  
 //float rangetest = requestRange();
 //Serial.print("Range Test:   "); Serial.print(range);

   
  // MAXSONAR SECTION
 takeRangeReading(); //Tell the sensor to perform a ranging cycle
 //Wait for sensor to finish
 delay(100);
 word MSrange = requestRange() * 1.5; //Get the range from the sensor

 //Serial.print("Range: "); Serial.print(MSrange); Serial.println(" cm."); //Print to the user
 
 
 //DISPLAY/ACTIVATE MOTORS SECTION

//  if(MSrange < T){
//    Serial.println("WARNING: OBSTACLE AHEAD.");
//  }


  currTime = millis();
  int side = 0;
  if(MSrange < stopdist || cmL < stopdist || cmR < stopdist){
    side = findMin(MSrange, cmL, cmR);
    if((currTime - prevTime) > stopdist/8){
       turnOn(side);
       prevTime = currTime;
    }
  }
  else if(MSrange < 2*stopdist || cmL < 2*stopdist || cmR < 2*stopdist){
    side = findMin(MSrange, cmL, cmR);
    if((currTime - prevTime) > (stopdist/4)){
       turnOn(side);
       prevTime = currTime;
    }
  }
  else if(MSrange < 3*stopdist || cmL < 3*stopdist || cmR < 3*stopdist){
    side = findMin(MSrange, cmL, cmR);
    if((currTime - prevTime) > (stopdist/2)){
       turnOn(side);
       prevTime = currTime;
    }
  }
  else{
    turnOffAll();
  }

  delay(100);
}


//Commands the sensor to take a range reading
void takeRangeReading(){
 Wire.beginTransmission(SensorAddress); //Start addressing
 Wire.write(RangeCommand); //send range command
 Wire.endTransmission(); //Stop and do something else now
}
//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication.
word requestRange(){
 Wire.requestFrom(SensorAddress, byte(2));
 if(Wire.available() >= 2){ //Sensor responded with the two bytes
 byte HighByte = Wire.read(); //Read the high byte back
 byte LowByte = Wire.read(); //Read the low byte back
 word range = word(HighByte, LowByte); //Make a 16-bit word out of the two bytes for the range
 return range;
 }
 else {
 return word(0); //Else nothing was received, return 0
 }
}

void changeAddress(byte oldAddress, byte newAddress, boolean SevenBitHuh){
 Wire.beginTransmission(oldAddress); //Begin addressing
 Wire.write(ChangeAddressCommand1); //Send first change address command
 Wire.write(ChangeAddressCommand2); //Send second change address command
 byte temp;
 if(SevenBitHuh){ temp = newAddress << 1; } //The new address must be written to the sensor
 else { temp = newAddress; } //in the 8bit form, so this handles automatic shifting

 Wire.write(temp); //Send the new address to change to
 Wire.endTransmission();
 
} 

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long durationOfPing(int pin){
  long duration;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);
  
  return duration;
}

int findMin(long mid, long left, long right){
  long minVal = mid;
  int side = 2;
  if(left < minVal) minVal = left;
  if(right < minVal) minVal = right;
  
  if(minVal == mid) side = 0;
  else if(minVal == left) side = -1;
  else if(minVal == right) side = 1;
  else side = 2;
  
  return side;
}

void turnOn(int side){
  if(side == -1){ //left
      digitalWrite(mPin_Right_UU, HIGH);
      digitalWrite(mPin_Right_UM, HIGH);
      digitalWrite(mPin_Right_LM, HIGH);
      digitalWrite(mPin_Right_LL, HIGH);
      digitalWrite(mPin_Center_UU, HIGH);
      digitalWrite(mPin_Center_UM, HIGH);
      digitalWrite(mPin_Center_LM, HIGH);
      digitalWrite(mPin_Center_LL, HIGH);
      digitalWrite(mPin_Left_UU, LOW);
      digitalWrite(mPin_Left_UM, LOW);
      digitalWrite(mPin_Left_LM, LOW);
      digitalWrite(mPin_Left_LL, LOW);
    }
    else if(side == 0){ //middle
      digitalWrite(mPin_Right_UU, HIGH);
      digitalWrite(mPin_Right_UM, HIGH);
      digitalWrite(mPin_Right_LM, HIGH);
      digitalWrite(mPin_Right_LL, HIGH);
      digitalWrite(mPin_Left_UU, HIGH);
      digitalWrite(mPin_Left_UM, HIGH);
      digitalWrite(mPin_Left_LM, HIGH);
      digitalWrite(mPin_Left_LL, HIGH);
      digitalWrite(mPin_Center_UU, LOW);
      digitalWrite(mPin_Center_UM, LOW);
      digitalWrite(mPin_Center_LM, LOW);
      digitalWrite(mPin_Center_LL, LOW);
    }
    else if(side == 1){ //right
      digitalWrite(mPin_Left_UU, HIGH);
      digitalWrite(mPin_Left_UM, HIGH);
      digitalWrite(mPin_Left_LM, HIGH);
      digitalWrite(mPin_Left_LL, HIGH);
      digitalWrite(mPin_Center_UU, HIGH);
      digitalWrite(mPin_Center_UM, HIGH);
      digitalWrite(mPin_Center_LM, HIGH);
      digitalWrite(mPin_Center_LL, HIGH);
      digitalWrite(mPin_Right_UU, LOW);
      digitalWrite(mPin_Right_UM, LOW);
      digitalWrite(mPin_Right_LM, LOW);
      digitalWrite(mPin_Right_LL, LOW);
    }
    else{
      digitalWrite(mPin_Left_UU, HIGH);
      digitalWrite(mPin_Left_UM, HIGH);
      digitalWrite(mPin_Left_LM, HIGH);
      digitalWrite(mPin_Left_LL, HIGH);
      digitalWrite(mPin_Center_UU, HIGH);
      digitalWrite(mPin_Center_UM, HIGH);
      digitalWrite(mPin_Center_LM, HIGH);
      digitalWrite(mPin_Center_LL, HIGH);
      digitalWrite(mPin_Right_UU, HIGH);
      digitalWrite(mPin_Right_UM, HIGH);
      digitalWrite(mPin_Right_LM, HIGH);
      digitalWrite(mPin_Right_LL, HIGH);
    }
}

void turnOffAll(){
      digitalWrite(mPin_Left_UU, HIGH);
      digitalWrite(mPin_Left_UM, HIGH);
      digitalWrite(mPin_Left_LM, HIGH);
      digitalWrite(mPin_Left_LL, HIGH);
      digitalWrite(mPin_Center_UU, HIGH);
      digitalWrite(mPin_Center_UM, HIGH);
      digitalWrite(mPin_Center_LM, HIGH);
      digitalWrite(mPin_Center_LL, HIGH);
      digitalWrite(mPin_Right_UU, HIGH);
      digitalWrite(mPin_Right_UM, HIGH);
      digitalWrite(mPin_Right_LM, HIGH);
      digitalWrite(mPin_Right_LL, HIGH);
}



