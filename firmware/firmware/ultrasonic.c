// HC-SR04 Ultrasonic Sensor
// TRIG = Pin 6
// ECHO = Pin 7

const int trigPin = 6;
const int echoPin = 7;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW); // ensure low at start
  Serial.println("Ultrasonic Sensor Ready...");
}

void loop() {
  long duration;
  float distance;

  // Send trigger pulse (10 microseconds)
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the echo pulse width
  duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout

  // If no echo received
  if (duration == 0) {
    Serial.println("Out of range");
  } else {
    // Convert time → distance
    distance = duration * 0.0343 / 2.0;  // SPEED OF SOUND → cm/µs
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(200); // small delay between readings
}
