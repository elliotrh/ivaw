// Parallax Ping Ultrasonic Sensor test code
// Axion Electronics
// Kyle Lam, John Efseaff
// 5V to 5V output, GND to GND, SIG to digital pin 7 (PWM)

// this constant won't change.  It's the pin number
// of the sensor's output:
const int pingPin = 31;

void setup() {
  // initialize serial communication for serial monitor
  Serial.begin(9600);
    pinMode(pingPin, OUTPUT);

}

void loop() {

  digitalWrite(pingPin, LOW);
  delay(1000);
  digitalWrite(pingPin, HIGH);
  delay(1000);
}

