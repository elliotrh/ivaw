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

void setup() {
 Serial.begin(115200); //Open serial connection at 9600? baud
 Wire.begin(); //Initiate Wire library for I2C communications with the I2CXL-MaxSonar-EZ

/*  #ifndef ESP8266
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
  uint32_t currentFrequency;
    */
  Serial.println("Hello!");
  
 
}

void loop(){

  // PARALLAX PING SECTION
  // data vars
  long durationL, inchesL, cmL, durationR, inchesR, cmR, threshold, avgL, avgR, T, d;
  int count = 0;
  //threshold = 10; //10cm ~ 4in
  //d = 10;

  //d == 6in ~ 15cm
  d = 15;

  avgL = 0;
  avgR = 0;
  
  while(count < 100){
  avgL += durationOfPing(pingPinL);
  avgR += durationOfPing(pingPinR);
  count++;
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



  
 //float rangetest = requestRange();
 //Serial.print("Range Test:   "); Serial.print(range);

   
  // MAXSONAR SECTION
 takeRangeReading(); //Tell the sensor to perform a ranging cycle
 delay(100); //Wait for sensor to finish
 word range = requestRange(); //Get the range from the sensor
 Serial.print("Range: "); Serial.print(range); Serial.println(" cm."); //Print to the user

 if(range < T){
  Serial.println("WARNING: OBSTACLE AHEAD.");
  }
  if(abs(threshold) <= T){
    Serial.println("middle");
  }
  if(threshold > T){
    Serial.println("left");
  }
  if(threshold < -T){
    Serial.println("right");
  }

  Serial.print("LeftSensor: ");
  Serial.print(inchesL);
  Serial.print("in, ");
  Serial.print(cmL);
  Serial.print("cm");
  Serial.println();

  Serial.print("RightSensor: ");
  Serial.print(inchesR);
  Serial.print("in, ");
  Serial.print(cmR);
  Serial.print("cm");
  Serial.println();
  Serial.println();

  delay(500);
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





