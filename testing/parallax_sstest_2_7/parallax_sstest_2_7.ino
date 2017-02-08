// 2 Parallax Ping Ultrasonic Sensors, side-by-side configuration
// Axion Electronics
// Kyle Lam
// 5V to 5V output, GND to GND, RSIG to digital pin 7 (PWM), LSIG to digital pin 8 (PWM)

// Distance between sensors should be 4 inches.

// two PWM pins
const int pingPinL = 7;
const int pingPinR = 11;

void setup() {
  // to print left/right/middle and other processed outputs
  Serial.begin(9600);
}

void loop() {
  // data vars
  long durationL, inchesL, cmL, durationR, inchesR, cmR;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPinL, OUTPUT);
  pinMode(pingPinR, OUTPUT);
  digitalWrite(pingPinL, LOW);
  digitalWrite(pingPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPinL, HIGH);
  digitalWrite(pingPinR, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPinL, LOW);
  digitalWrite(pingPinR, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPinL, INPUT);
  pinMode(pingPinR, INPUT);
  durationL = pulseIn(pingPinL, HIGH);
  durationR = pulseIn(pingPinR, HIGH);

  // convert the time into a distance
  inchesL = microsecondsToInches(durationL);
  inchesR = microsecondsToInches(durationR);
  cmL = microsecondsToCentimeters(durationL);
  cmR = microsecondsToCentimeters(durationR);

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
