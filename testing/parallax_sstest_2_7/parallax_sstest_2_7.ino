// 2 Parallax Ping Ultrasonic Sensors, side-by-side configuration
// Axion Electronics
// Kyle Lam
// 5V to 5V output, GND to GND, LSIG to digital pin 7 (PWM), RSIG to digital pin 6 (PWM)

// Distance between sensors should be 4 inches.

// two PWM pins
const int pingPinL = 6;
const int pingPinR = 7;
const int motorPin = 31;

void setup() {
  // to print left/right/middle and other processed outputs
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);

}

void loop() {
  // data vars
  long durationL, inchesL, cmL, durationR, inchesR, cmR, threshold, avgL, avgR, T, d;
  int count = 0;
  //threshold = 10; //10cm ~ 4in
  //d = 10;

  //d == 6in ~ 15cm
  d = 25.5;
  avgL = 0;
  avgR = 0;
  
  while(count < 50){
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

  if(abs(threshold) <= T){
    Serial.println("middle");
    digitalWrite(motorPin, LOW);
  }
  else{
    digitalWrite(motorPin, HIGH);
  }
  if(threshold > T){
    Serial.println("left");
  }
  if(threshold < -T){
    Serial.println("right");
  }

  if(cmR > T/2 && cmL > T/2){
    Serial.println("no object");
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

  delay(100);
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


