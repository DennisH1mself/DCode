/*
  BrOzones DemoSat Spring 2024 Payload - High Altitude Ozone Depletion Refrigerant Analysis
                                        TERRESTRIAL TESTING SOURCE
  Authors:
    Koko Stathopulos
    Marco Villasuso
*/


/*
PROBLEM CODES (AND LED FLASH PATTERN):
  1: I2C ERROR ON 0x73 (flashes yellow)
  2: I2C ERROR ON 0x72 (flashes green)
  3: SERVO CONNECTION FAILURE (all lights flashing at the same time)
  4: EXTERNAL OZONE RUNTIME SENSOR FAILURE (red)
  5: INTERNAL OZONE RUNTIME SENSOR (blue)
  6: OTHER (Red/Blue and Green/Yellow alternating)

pins:
  // RED: 2
  // BLUE: 3
  // GREEN: 4
  // YELLOW: 5
*/
#define FLASH_DELAY 250 // Time in milliseconds between when an LED flashes if there's a problem code

#include <DFRobot_OzoneSensor.h> // Ozone sensor library by DFRobot (https://codeload.github.com/DFRobot/DFRobot_OzoneSensor/zip/master)
#include <Servo.h> // Servo library by Arduino (https://www.arduino.cc/reference/en/libraries/servo/)

#define COLLECT_NUMBER 20  //what is this? <-> collect number, the collection range is 1-100
#define AREF 5.0

#define OZONE1_ADDR OZONE_ADDRESS_3  //ozone1 at i2c address 0x73
#define OZONE2_ADDR OZONE_ADDRESS_2  //ozone2 at i2c address 0x72
#define LIGHTER_SERVO 10
#define ATM_SERVO 11

// declare sensor classes
DFRobot_OzoneSensor Ozone1;
DFRobot_OzoneSensor Ozone2;

Servo gasServo; // Connection J13
Servo atmServo; // Connection J10

// Variables used by pressure sensor
int pressure;
float pressureVolt;
float psi;

// Variables used by temperature sensor
int temp1;
float temp1Volt;
float temp1C;
float temp1F;

uint32_t timeStamp; // Timestamp in milliseconds since program started

uint8_t problemCode; // If there's a problem, the entire program haults and throws a problem code
uint8_t EXTsensorRuntimeTries; // Counts how many times the exterior o3 sensor reads 0
uint8_t INTsensorRuntimeTries; // Counts how many times the interior o3 sensor reads 0

inline float getAltitude(float psi) { // Equation to convert pressure to altitude with around +- 1,000 ft of precision. Credit to Dennis for this
  return ((1-pow(((psi*68.9476)/1013.25),0.190284))*145366.45);
}

inline void moveServos(int gasAngle, int atmAngle) { // A safe function that first makes sure the servos are working before moving them, if not throw a problem code
   if (!gasServo.attached() || !atmServo.attached()) {
    Serial.println("SERVO ATTACHMENT FAILURE: ");
    Serial.println(gasServo.attached());
    Serial.println(atmServo.attached());
    Serial.println("-- END SERVO DIAG.");
    problemCode = 3; 
    return;
  }

  gasServo.write(gasAngle);
  atmServo.write(atmAngle);
}

inline void flashLED(uint32_t pinsToFlash) { // The function used to flash an LED pattern that correlates to one of the problem codes listed at the top
// More memory efficient way of storing LED flash patterns, better than calling a function 20 times a second on this poor little arduino uno.
// All this does is read off bytes of an integer, 1 at a time, and if a byte is 0xFF (a comlpete byte), then flash that LED
  uint8_t firstPin = pinsToFlash & 0xFF; 
  uint8_t secondPin = (pinsToFlash >> 8) & 0xFF;
  uint8_t thirdPin = (pinsToFlash >> 16) & 0xFF;
  uint8_t fourthPin = (pinsToFlash >> 24) & 0xFF;
  
  // RED: 2
  // BLUE: 3
  // GREEN: 4
  // YELLOW: 5
  Serial.print("FLASHING LED: ");
  Serial.println(pinsToFlash, HEX);

  if (firstPin == 0xFF)
    digitalWrite(2, LOW);
  if (secondPin == 0xFF)
    digitalWrite(3, LOW);
  if (thirdPin == 0xFF)
    digitalWrite(4, LOW);
  if (fourthPin == 0xFF)
    digitalWrite(5, LOW);

  delay(FLASH_DELAY);

   if (firstPin > 0)
    digitalWrite(2, HIGH);
  if (secondPin > 0)
    digitalWrite(3, HIGH);
  if (thirdPin > 0)
    digitalWrite(4, HIGH);
  if (fourthPin > 0)
    digitalWrite(5, HIGH);

  delay(FLASH_DELAY);
}

int delayTimer = 0;
bool sentMessage = false;

void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);
  // Init LED array
  pinMode(2, OUTPUT); // RED LED
  pinMode(3, OUTPUT); // BLUE LED
  pinMode(4, OUTPUT); // GREEN LED
  pinMode(5, OUTPUT); // YELLOW LED
  pinMode(6, OUTPUT); // ORANGE LED (POWER INDICATOR)

  digitalWrite(6, HIGH);  // Turn on orange LED to indicate power is on

  // Variable inititalization
  problemCode = 0;
  timeStamp = 0;
  EXTsensorRuntimeTries = 0;
  INTsensorRuntimeTries = 0;

  uint8_t tryCount = 0; // The amount of times to try connections before the board throws a problemCode

  while (!Ozone1.begin(OZONE1_ADDR)) { // Attempt to connect the external ozone sensor
    Serial.println("I2c device number error on 0x73 (EXTERNAL OZONE)");
    tryCount++;
    if (tryCount > 10) {
      problemCode = 1;
      return;
    }
    delay(1000);
  }
  while (!Ozone2.begin(OZONE2_ADDR)) { // Attempt to connect the internal ozone sensor
    Serial.println("I2c device number error on 0x72 (INTERNAL OZONE)");
    tryCount++;
    if (tryCount > 10) {
      problemCode = 2;
      return;
    }
    delay(1000);
  }
  Serial.println("I2c connect success !");

  Ozone1.setModes(MEASURE_MODE_PASSIVE);
  Ozone2.setModes(MEASURE_MODE_PASSIVE);
  /*   Set iic mode, active mode or passive mode
       MEASURE_MODE_AUTOMATIC            // active  mode
       MEASURE_MODE_PASSIVE              // passive mode
  */
  Serial.println("Connecting servos...");
  gasServo.attach(LIGHTER_SERVO); // Try connecting servos
  atmServo.attach(ATM_SERVO);
  if (!gasServo.attached() || !atmServo.attached()) { // If servos aren't connected, throw a problem code
    Serial.println("SERVO ATTACHMENT FAILURE: ");
    Serial.println(gasServo.attached());
    Serial.println(atmServo.attached());
    Serial.println("-- END SERVO DIAG.");
    problemCode = 3; 
    return;
  }
  gasServo.write(0); // Close lighter valve
  atmServo.write(90); // Open atmosphere valve
  Serial.println("Servos connected successfully");

  Serial.println("Time,TempF,Pres,IntO3,ExtO3");
}

void loop() {
  if (problemCode > 0) { // Detects if there's a problem code thrown, and flash the accompanying LED pattern
      uint32_t flashCode = 0x00000000;
      switch (problemCode) {
        case 1:
          flashCode |= 0xFF000000;
        break;

        case 2:
          flashCode |= 0x00FF0000;
        break;

        case 3:
          flashCode |= 0xFFFFFFFF;
        break;

        case 4: 
          flashCode |= 0x000000FF;
        break;

        case 5:
          flashCode |= 0x0000FFFF;
        break;

        default:
          flashCode = 0xFFFF0000;
          flashLED(flashCode);
          delay(500);
          Serial.println("UNKOWN ERRROR FLASH CODE");
          flashCode = 0x0000FFFF;
        break;
      }
      flashLED(flashCode);
      delay(500);
      if (flashCode != 0xFFFF) { // make an exception for flash code 0xFFFF (Runtime ozone sensors), this is because there's a chance this problem corrects itself and we can continue on with the experiment.
        return;
      }
  }

  digitalWrite(3, HIGH); // Power blue LED : read internal ozone data (these are not related to flash codes, but are to indicate what data is being written/read)
  int16_t intO3 = Ozone2.readOzoneData(COLLECT_NUMBER);  // range 1-100
  digitalWrite(2, HIGH); // Power yellow LED: read external ozone data
  int16_t extO3 = Ozone1.readOzoneData(COLLECT_NUMBER);  // range 1-100

  // Read temperature sensor
  temp1 = analogRead(A3);
  temp1Volt = temp1 * (AREF / 1023.0);  // 5v for default aref
  temp1C = (temp1Volt - 0.5) / (0.01);
  temp1F = (temp1C * (9.0 / 5.0) + 32.0);
  digitalWrite(4, HIGH);

  // Read pressure snesor
  pressure = analogRead(A0);
  pressureVolt = pressure * (AREF / 1023.0);  //5v for dfault aref
  psi = (pressureVolt - 0.5) * (15.0 / 4.0);
  digitalWrite(5, HIGH);

  // Log the time
  timeStamp = millis();
  Serial.print(timeStamp);

  Serial.print(",");
  Serial.print(temp1F);
  digitalWrite(4, LOW);

  Serial.print(",");
  Serial.print(psi);
  digitalWrite(5, LOW);

  Serial.print(",");
  Serial.print(intO3);
  digitalWrite(2, LOW);

  Serial.print(",");
  Serial.println(extO3);
  digitalWrite(3, LOW);

  // Remember that problem code exception from earlier (0xFFFF)? This is where we detect if a sensor has a problem, or remove the problem code if the problem fixes itself
  if (extO3 == 0) {
    EXTsensorRuntimeTries++;
    if (EXTsensorRuntimeTries > 20)
      problemCode = 4;
  } else if (extO3 >= 20) {
    EXTsensorRuntimeTries = 0;
    if (problemCode == 4) {
      problemCode = 0;
    }
  }

  if (intO3 == 0) {
      INTsensorRuntimeTries++;
      if (INTsensorRuntimeTries > 20)
      problemCode = 5;
  } else if (intO3 >= 20) {
    INTsensorRuntimeTries = 0;
    if (problemCode == 5) {
      problemCode = 0;
    }
  }

  delay(500);  //Amount of time between samples (milliseconds)

  delayTimer++;
  if (delayTimer > 600) { // Swap valves and let r1234yf into the test chamber after 5 minutes. (delayTimer increases by 2 every second)
    if (!sentMessage)
      Serial.println("CLOSING ATMOSPHERE VALVE");
     atmServo.write(0); // Open atmosphere valve
     delay(500); // allow the atmosphere valve to close fully before injecting r1234yf
     if (!sentMessage)
        Serial.println("OPENING GAS VALVE");
    gasServo.write(90); // Close lighter valve
    sentMessage = true; // This code runs twice a second to make sure the valves stay locked where they are the entire time, use the sentMessage to shut the "OPENING GAS VALVE..." up after the first time it prints
 
  }
}
