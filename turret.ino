// arduino_switch_example.ino

#include <Servo.h>

Servo panServo, tiltServo;
int pan = 1500;
int tilt = 1500;
const uint8_t HEADER = 0xFF;

void setup() {
  Serial.begin(500000);
  panServo.attach(9, 1000, 2000);
  tiltServo.attach(8, 1000, 2000);
  panServo.writeMicroseconds(pan);
  tiltServo.writeMicroseconds(tilt);
}
void loop() {
  // If at least 3 bytes are waiting...
  if (Serial.available() >= 3) {
    // Check for our header byte
    if (Serial.peek() == HEADER) {
      Serial.read();  // consume 0xFF

      // Read raw bytes
      uint8_t rawPan  = Serial.read();
      uint8_t rawTilt = Serial.read();

      // Reinterpret as signed offsets
      int8_t panOff  = int8_t(rawPan);
      int8_t tiltOff = int8_t(rawTilt);

      // Map to 1000–2000 µs pulses around 1500
      pan += panOff;
      tilt += tiltOff;
      panServo.writeMicroseconds(pan);
      tiltServo.writeMicroseconds(tilt);

    } else {
      // If no header, drop one byte and retry
      Serial.read();
    }
  }
}