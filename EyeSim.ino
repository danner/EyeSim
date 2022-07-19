/*
    EyeSim using ServoEasing

    Based on the TwoServo.cpp example in the ServoEasing Library. <https://github.com/ArminJo/ServoEasing/tree/master/examples/TwoServos>

    ServoEasing is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
*/

#include <Arduino.h>

// Must specify this before the include of "ServoEasing.hpp"
#define USE_PCA9685_SERVO_EXPANDER    // Activate this to enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB                 // Activate this to force additional using of regular servo library.
//#define USE_LEIGHTWEIGHT_SERVO_LIB    // Makes the servo pulse generating immune to other libraries blocking interrupts for a longer time like SoftwareSerial, Adafruit_NeoPixel and DmxSimple.
//#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activate this to disable all but LINEAR movement. Saves up to 1540 bytes program memory.
#define DISABLE_COMPLEX_FUNCTIONS     // Activate this to disable the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
//#define MAX_EASING_SERVOS 2
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables constraints. Saves 4 bytes RAM per servo but strangely enough no program memory.
//#define DEBUG                         // Activate this to generate lots of lovely debug output for this library.

//#define PRINT_FOR_SERIAL_PLOTTER      // Activate this to generate the Arduino plotter output from ServoEasing.hpp.
#include "ServoEasing.hpp"

#if !defined(PRINT_FOR_SERIAL_PLOTTER)
#define INFO // to see serial text output for loop
#endif

#include "PinDefinitionsAndMore.h"
/*
 * Pin mapping table for different platforms - used by all examples
 *
 * Platform         Servo1      Servo2      Servo3      Analog     Core/Pin schema
 * -------------------------------------------------------------------------------
 * (Mega)AVR + SAMD    9          10          11          A0
 * ATtiny3217         20|PA3       0|PA4       1|PA5       2|PA6   MegaTinyCore
 * ESP8266            14|D5       12|D6       13|D7        0
 * ESP32               5          18          19          A0
 * BluePill          PB7         PB8         PB9         PA0
 * APOLLO3            11          12          13          A3
 * RP2040             6|GPIO18     7|GPIO19    8|GPIO20
 */

#if defined(USE_PCA9685_SERVO_EXPANDER)
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS); 
ServoEasing Servo2(PCA9685_DEFAULT_ADDRESS);
#else
ServoEasing Servo1;
ServoEasing Servo2;
#endif
#define START_DEGREE_VALUE  0+90 // The degree value written to the servo at time of attach.
#define MAX_X 60+90
#define MIN_X -60+90
#define MAX_Y 25+90
#define MIN_Y -40+90
#define MAX_SPEED 40
#define MIN_SPEED 5
#define LONGEST_WAIT 1000*60*1/2 // 30 seconds

// Define my functions
void blinkLED();
void push_position(int x, int y, int x_speed, int y_speed);
void stare();
void micromovement();
void previous();
void quick();
void slow();
void big_x();
void big_y();
void test_extents();

// Define my variables
int curr_x = START_DEGREE_VALUE;
int curr_y = START_DEGREE_VALUE;
int prev_x = START_DEGREE_VALUE;
int prev_y = START_DEGREE_VALUE;
int curr_x_speed = MIN_SPEED;
int curr_y_speed = MIN_SPEED;
int prev_x_speed = MIN_SPEED;
int prev_y_speed = MIN_SPEED;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_SERVO_EASING));
#endif

    /********************************************************
     * Attach servo to pin and set servos to start position.
     * This is the position where the movement starts.
     *******************************************************/
#if defined(USE_PCA9685_SERVO_EXPANDER)
    if (Servo1.InitializeAndCheckI2CConnection(&Serial)) {
        while (true) {
            blinkLED();
        }
    }
    if (Servo2.InitializeAndCheckI2CConnection(&Serial)) {
        while (true) {
            blinkLED();
        }
    }
#endif
#if !defined(PRINT_FOR_SERIAL_PLOTTER)
#  if defined(USE_PCA9685_SERVO_EXPANDER)
#undef SERVO1_PIN
#define SERVO1_PIN  0 // we use first port of expander
    Serial.println(F("Attach servo to port 0 of PCA9685 expander"));
#undef SERVO2_PIN
#define SERVO2_PIN  1 // we use second port of expander
    Serial.println(F("Attach servo to port 1 of PCA9685 expander"));
#  else
    Serial.println(F("Attach servo at pin " STR(SERVO1_PIN)));
    Serial.println(F("Attach servo at pin " STR(SERVO2_PIN)));
#  endif
#endif
    if (Servo1.attach(SERVO1_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            blinkLED();
        }
    }
    if (Servo2.attach(SERVO2_PIN, START_DEGREE_VALUE) == INVALID_SERVO) {
        Serial.println(F("Error attaching servo"));
        while (true) {
            blinkLED();
        }
    }

    // Wait for servo to reach start position.
    delay(500);
#if defined(PRINT_FOR_SERIAL_PLOTTER)
    // Legend for Arduino Serial plotter
    Serial.println(); // end of line of attach values
    Serial.println("OneServo[us]_Linear->Cubic->Linear");
#endif
}

void blinkLED() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void push_position(int x, int y, int x_speed, int y_speed) {

//constrain the y value to get more high values (looking down)
if (y < (MIN_Y/2+MAX_Y)/2) {
  y = 2 * y - MIN_Y;
} else {
  y = 0.5 * y + (MAX_Y / 2);
}

#if defined(INFO)
  Serial.print("x:");
  Serial.print(x);
  Serial.print(" y:");
  Serial.print(y);
  Serial.print(" x_speed:");
  Serial.print(x_speed);
  Serial.print( " y_speed:");
  Serial.println(y_speed);
#endif
  prev_x = curr_x;
  prev_y = curr_y;
  curr_x = constrain(x, MIN_X, MAX_X);
  curr_y = constrain(y, MIN_Y, MAX_Y);
  prev_x_speed = curr_x_speed;
  prev_y_speed = curr_y_speed;
  curr_x_speed = constrain(x_speed, MIN_SPEED, MAX_SPEED);
  curr_y_speed = constrain(y_speed, MIN_SPEED, MAX_SPEED);
}

void stare() {
#if defined(INFO)
  Serial.println(F("stare"));
#endif
  delay(random(1000, LONGEST_WAIT));
}

void micromovement() {
#if defined(INFO)
  Serial.println(F("micromovement"));
#endif
  push_position(
    random(curr_x - 5, curr_x + 5),
    random(curr_y - 5, curr_y + 5),
    random(MAX_SPEED / 2, MAX_SPEED),
    curr_y_speed);
  Servo1.setEaseTo(curr_x, curr_x_speed);
  // second servo takes speed from first for this move so they finish at the same time?
  Servo2.startEaseToD(curr_y, Servo1.mMillisForCompleteMove); // This start interrupt for all servos
  // blink until both servos stop
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(random(300, 1000));
}

void previous() {
#if defined(INFO)
  Serial.println(F("previous"));
#endif
  push_position(
    prev_x,
    prev_y,
    prev_x_speed,
    prev_y_speed);
  Servo1.setEaseTo(curr_x, curr_x_speed);
  Servo2.startEaseTo(curr_y, curr_y_speed); // This start interrupt for all servos
  // blink until both servos stop
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(random(300, 1000));
}

void quick() {
#if defined(INFO)
  Serial.println(F("quick"));
#endif
  push_position(
    random(MIN_X, MAX_X),
    random(MIN_Y, MAX_Y),
    random(MAX_SPEED / 2, MAX_SPEED),
    random(MAX_SPEED / 2, MAX_SPEED));
  Servo1.setEaseTo(curr_x, curr_x_speed);
  // second servo takes speed from first for this move so they finish at the same time?
  Servo2.startEaseToD(curr_y, Servo1.mMillisForCompleteMove); // This start interrupt for all servos
  // blink until both servos stop
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(random(1000, 2000));
}

void slow() {
#if defined(INFO)
  Serial.println(F("slow"));
#endif
  Servo1.setEasingType(EASE_LINEAR);
  push_position(
    random(MIN_X, MAX_X),
    random(MIN_Y, MAX_Y),
    random(MIN_SPEED, MAX_SPEED / 3),
    random(MIN_SPEED, MAX_SPEED / 3));
  Servo1.setEaseTo(curr_x, curr_x_speed);
  // second servo takes speed from first for this move so they finish at the same time?
  Servo2.startEaseToD(curr_y, Servo1.mMillisForCompleteMove); // This start interrupt for all servos
  // blink until both servos stop
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  Servo1.setEasingType(EASE_CUBIC_IN_OUT);
  delay(random(1000, 2000));
}


void big_x() {
#if defined(INFO)
  Serial.println(F("big x"));
#endif
  if (curr_x < START_DEGREE_VALUE) {
    push_position(
      random(START_DEGREE_VALUE, MAX_X),
      curr_y,
      random(MIN_SPEED, MAX_SPEED),
      random(MIN_SPEED, MAX_SPEED));
  } else {
    push_position(
      random(MIN_X, START_DEGREE_VALUE),
      curr_y,
      random(MIN_SPEED, MAX_SPEED),
      random(MIN_SPEED, MAX_SPEED));
  }
  Servo1.setEaseTo(curr_x, curr_x_speed);
  // second servo takes speed from first for this move so they finish at the same time?
  Servo2.startEaseTo(curr_y, curr_y_speed); // This start interrupt for all servos
  // blink until both servos stop
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(random(1000, 2000));
}
void big_y() {
#if defined(INFO)
  Serial.println(F("big y"));
#endif
  if (curr_y < START_DEGREE_VALUE) {
    push_position(
      curr_x,
      random(START_DEGREE_VALUE, MAX_Y),
      random(MIN_SPEED, MAX_SPEED),
      random(MIN_SPEED, MAX_SPEED));
  } else {
    push_position(
      curr_x,
      random(MIN_Y, START_DEGREE_VALUE),
      random(MIN_SPEED, MAX_SPEED),
      random(MIN_SPEED, MAX_SPEED));
  }
  Servo1.setEaseTo(curr_x, curr_x_speed);
  // second servo takes speed from first for this move so they finish at the same time?
  Servo2.startEaseTo(curr_y, curr_y_speed); // This start interrupt for all servos
  // blink until both servos stop
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(random(1000, 2000));
}
void test_extents() {
#if defined(INFO)
  Serial.println(F("center"));
#endif
  Servo1.startEaseTo(START_DEGREE_VALUE, MIN_SPEED);
  Servo2.startEaseTo(START_DEGREE_VALUE, MIN_SPEED);
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(1000);

#if defined(INFO)
  Serial.println(F("lower left"));
#endif
  Servo1.startEaseTo(MIN_X, MIN_SPEED);
  Servo2.startEaseTo(MIN_Y, MIN_SPEED);
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(500);

#if defined(INFO)
  Serial.println(F("lower right"));
#endif
  Servo1.startEaseTo(MAX_X, MIN_SPEED);
  Servo2.startEaseTo(MIN_Y, MIN_SPEED);
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(500);

#if defined(INFO)
  Serial.println(F("upper right"));
#endif
  Servo1.startEaseTo(MAX_X, MIN_SPEED);
  Servo2.startEaseTo(MAX_Y, MIN_SPEED);
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(500);

#if defined(INFO)
  Serial.println(F("upper right"));
#endif
  Servo1.startEaseTo(MIN_X, MIN_SPEED);
  Servo2.startEaseTo(MAX_Y, MIN_SPEED);
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(500);

#if defined(INFO)
  Serial.println(F("center"));
#endif
  Servo1.startEaseTo(START_DEGREE_VALUE, MIN_SPEED);
  Servo2.startEaseTo(START_DEGREE_VALUE, MIN_SPEED);
  while (ServoEasing::areInterruptsActive()) {
    blinkLED();
  }
  delay(500);
};

void loop() {

  /*
     Move both servos blocking
  */
  // #if defined(INFO)
  //     Serial.println(F("Move to 0/90 degree with 30 degree per second blocking"));
  // #endif
  //     setSpeedForAllServos(30);
  //     Servo1.setEaseTo(0);
  //     Servo2.setEaseTo(90);
  //     synchronizeAllServosStartAndWaitForAllServosToStop();

  //possible choices with probability numbers
  // stare
  // micromovement
  // previous position at same speed (saccade)
  // quick move (context shift)
  // slow move (following)
  // big x (shift pan)
  // big y (shift tilt)
  
//  test_extents();
  switch (random(16)) {
    case 0:
    case 1:
      micromovement();
      break;
    case 2:
    case 3:
    case 4:
      slow();
      break;
    case 5:
    case 6:
    case 7:
      previous();
      break;
    case 8:
      quick();
      break;
    case 9:
      big_x();
      break;
    case 10:
    case 11:
      big_y();
      break;
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
      stare();
      break;
  }
}
