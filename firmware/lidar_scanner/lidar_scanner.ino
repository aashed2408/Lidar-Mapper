/*
  LiDAR Scanner: a VL53L1X laser distance sensor on a servo.

  The servo sweeps the sensor from 0 to 180 degrees and back, in 2-degree steps.
  After each step the ESP32 takes one distance reading and sends it to the laptop
  over USB as one line of text:

      angle_degrees,distance_mm,ok        for example:  90,1234,1

  ok = 1 if the reading is valid, 0 if not. After each sweep it sends:  SWEEP_END
  The LiDAR Mapper website ("Real sensor" mode) reads these lines and builds a map.

  ---------------------------------------------------------------------------
  HOW TO UPLOAD THIS WITH THE ARDUINO IDE
  ---------------------------------------------------------------------------
  1. Install the Arduino IDE (version 2) from arduino.cc.
  2. Add ESP32 support:
       File > Preferences > "Additional boards manager URLs", paste:
         https://espressif.github.io/arduino-esp32/package_esp32_index.json
       Then Tools > Board > Boards Manager, search "esp32", install
       "esp32 by Espressif Systems".
  3. Install the two libraries:
       Tools > Manage Libraries, search and install
         "Adafruit_VL53L1X"  (by Adafruit)
         "ESP32Servo"        (by Kevin Harrington)
  4. Plug the ESP32 into the computer with a USB-C DATA cable.
     If no port shows up, install the CP2102 driver
     ("CP210x USB to UART Bridge VCP driver" from silabs.com) and try again.
  5. Tools > Board > esp32 > "ESP32 Dev Module".
     Tools > Port > pick the new port (COM3, /dev/cu.SLAB_USBtoUART, ...).
  6. Open this file and click Upload (the right arrow).
     If it gets stuck on "Connecting....", hold the BOOT button on the board
     until the upload starts.
  7. To check it works: Tools > Serial Monitor, set it to 115200 baud.
     You should see lines like "90,1234,1".
     IMPORTANT: close the Serial Monitor before using the website,
     because only one program can use the USB port at a time.

  Wiring:
    VL53L1X VIN -> ESP32 3V3     VL53L1X GND -> ESP32 GND
    VL53L1X SDA -> GPIO21        VL53L1X SCL -> GPIO22
    Servo signal (orange) -> GPIO18
    Servo power  (red)    -> VIN (5 V from USB)
    Servo ground (brown)  -> GND
*/

#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <ESP32Servo.h>

// ----- Pins -----
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int SERVO_PIN = 18;

// ----- Settings -----
const int STEP_DEGREES = 2;     // move 2 degrees between readings
const int SETTLE_MS = 40;       // wait this long after each move so the servo stops shaking
const int TIMING_BUDGET_MS = 50; // how long the sensor spends on each measurement
const int START_WAIT_MS = 5000; // time to attach the servo arm at 90 degrees

Adafruit_VL53L1X sensor;
Servo servo;

// Take one distance reading. Returns the distance in mm, and sets ok to
// true if the reading is valid.
int takeReading(bool &ok) {
  // Throw away any old reading taken while the servo was still moving.
  sensor.clearInterrupt();

  // Wait for a fresh reading (give up after 300 ms).
  unsigned long startTime = millis();
  while (!sensor.dataReady()) {
    if (millis() - startTime > 300) {
      ok = false;
      return 0;
    }
    delay(1);
  }

  // distance() gives -1 if the sensor says the reading is bad
  // (too far, too dark, or a weak signal).
  int mm = sensor.distance();
  sensor.clearInterrupt();
  ok = (mm > 0);
  return ok ? mm : 0;
}

// Move the servo to one angle, wait, take a reading and send it to the laptop.
void readAt(int angle) {
  servo.write(angle);
  delay(SETTLE_MS);
  bool ok;
  int mm = takeReading(ok);
  // One line: angle_degrees,distance_mm,ok
  Serial.print(angle);
  Serial.print(",");
  Serial.print(mm);
  Serial.print(",");
  Serial.println(ok ? 1 : 0);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("LiDAR scanner starting");

  // The servo: 50 pulses per second, 500-2400 microseconds covers 0-180 degrees.
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(90); // point straight ahead

  // Start the I2C bus and look for the sensor.
  Wire.begin(SDA_PIN, SCL_PIN);
  while (!sensor.begin(0x29, &Wire)) {
    // Not found: keep telling the user every second.
    Serial.println("ERROR: VL53L1X sensor not found. Check the wiring: "
                   "VIN->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22.");
    delay(1000);
  }
  Serial.println("VL53L1X sensor found");

  // Long distance mode (2 = long, up to about 4 m) and a 50 ms timing budget.
  // The distance mode must be set before the timing budget.
  sensor.VL53L1X_SetDistanceMode(2);
  sensor.setTimingBudget(TIMING_BUDGET_MS);
  sensor.VL53L1X_SetInterMeasurementInMs(TIMING_BUDGET_MS); // measure again straight away
  sensor.startRanging();

  // Time to attach the servo arm pointing straight ahead.
  Serial.println("Servo at 90 degrees. Attach the arm pointing straight ahead. Sweeping in 5 seconds...");
  delay(START_WAIT_MS);

  // Go to the starting angle. This is a big move, so give it half a second.
  servo.write(0);
  delay(500);
}

void loop() {
  // Sweep from 0 to 180 degrees.
  for (int angle = 0; angle <= 180; angle += STEP_DEGREES) readAt(angle);
  Serial.println("SWEEP_END");

  // And back from 180 to 0 degrees.
  for (int angle = 180; angle >= 0; angle -= STEP_DEGREES) readAt(angle);
  Serial.println("SWEEP_END");
}
