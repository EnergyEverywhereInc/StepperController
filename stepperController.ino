/**
 * A multi-axis stepper motor controller.
 * 
 * This controller do basic controls with 1-3 stepper motors.
 * 
 * Written by Mark Reynolds for Energy Everywhere in November of 2017.
 */

/*************************************************************/
/*               Serial Communication Protocol               */
/*************************************************************/

/***
 *** Intro 
 ***
 * 
 * This controller communicates entirely over a basic serial
 * protocol. It assumes a baud rate of 9600, 8 data bits, no
 * parity bits, and on stop bit. These are the default settings
 * for arduinos.
 */

/***
 *** Command Structure
 ***
 * 
 * Commands are made up of three parts: an action, an axis,
 * and additional action specific arguments.
 * 
 * The action and axis are always one character each.
 * 
 * The axis MUST always be 0-2 and be represented as an ASCII
 * character.
 * 
 * There should not be any characters in between each part.
 * 
 * Examples of valid commands:
 * 
 * Move 255 steps forward along axis 1:
 * +0255
 * 
 * Move 1437 steps backware along axis 3:
 * -21437
 * 
 * Get home switch status for axis 2:
 * H1
 */

/***
 *** Response Structure
 ***
 * 
 * Responses are made up of two parts: a status code and
 * additional action specific results.
 * 
 * The status code and any additional results will be separated
 * by a space.
 * 
 * Examples of valid responses:
 * 
 * Command succeeded:
 * 0
 * 
 * Command succeeded and result is true;
 * 0 T
 */

/***
 *** Command Reference
 ***
 * 
 ** Move Forward
 * 
 * Steps forward the desired distance.
 * 
 * Action Character: '+'
 * 
 * Additional arguements:
 *   - amount: An unsigned long for the amount of steps to move
 *   
 * Example:
 * 
 * * Move 255 steps forward along axis 1:
 * +0255
 * 
 * 
 ** Move Backward
 * 
 * Steps backward the desired distance.
 * 
 * Action Character: '-'
 * 
 * Additional arguements:
 *   - amount: An unsigned long for the amount of steps to move
 *   
 * Example:
 * 
 * Move 1437 steps backware along axis 3:
 * -21437
 * 
 * 
 ** Home Switch Status
 * 
 * Gets whether or not on the given axis the home sensor is
 * being tripped. Indicating on that axis, the controller is
 * at zero.
 * 
 * Action Character: 'H'
 * 
 * Additional results:
 *   - status: Either 'T' for true or 'F' for false.
 *   
 * Example:
 * 
 * Get home switch status for axis 2:
 * H1
 * 
 * Example Response:
 * 
 * Home Switch is active;
 * 0 T
 * 
 * 
 ** Get Axises
 * 
 * Gets the number of axises this controller controls. This
 * action is special because no axis is associated with it.
 * 
 * Action Character: 'A'
 * 
 * Additional results:
 *   - axises: A ascii number between 1-3.
 * 
 * 
 * * Get the number of axises.
 * A
 * 
 * Example Response:
 * 
 * Two axis controller;
 * 0 2
 */ 

/***
 *** Response Status Code Reference
 ***
 * 
 * 0 - Command Successful
 * 1 - Unknown Command.
 * 2 - Invalid Axis.
 */


/*************************************************************/
/*                  Internal Documentation                   */
/*************************************************************/

/**
 * Steps the motor forward the desired amount.
 * 
 * @param axis The axis to move along.
 * @param amt  The amount to move along this axis. In steps.
 * 
 * @pre axis must be between 0-2 inclusive.
 */
void moveForward(int axis, unsigned long amt);

/**
 * Steps the motor backward the desired amount.
 * 
 * @param axis The axis to move along.
 * @param amt  The amount to move along this axis. In steps.
 * 
 * @pre axis must be between 0-2 inclusive.
 */
void moveBackward(int axis, unsigned long amt);

/**
 * Checks if the home switch is being triggered.
 * 
 * @param axis The axis to check the home switch for.
 * 
 * @pre axis must be between 0-2 inclusive.
 * 
 * @return The state of the home switch.
 * 
 * @note The home pins should probably be connected to pull-down resistors.
 */
bool isHome(int axis);

#define STEP_PIN_0 13  // Digital output pin connected to EasyDriver Step (STEP) pin for axis 1.
#define DIR_PIN_0  4   // Digital output pin connected to EasyDriver Direction (DIR) pin for axis 1.
#define STEP_PIN_1 0   // Digital output pin connected to EasyDriver Step (STEP) pin for axis 2.
#define DIR_PIN_1  0   // Digital output pin connected to EasyDriver Direction (DIR) pin for axis 2.
#define STEP_PIN_2 0   // Digital output pin connected to EasyDriver Step (STEP) pin for axis 3.
#define DIR_PIN_2  0   // Digital output pin connected to EasyDriver Direction (DIR) pin for axis 3.

#define HOME_PIN_0 0   // Digital input pin connected to the home switch for axis 1.
#define HOME_PIN_1 0   // Digital input pin connected to the home switch for axis 2.
#define HOME_PIN_2 0   // Digital input pin connected to the home switch for axis 3.

#define AXISES     1  // Number of axises this controller is connected to.

#if (AXISES > 3) || (AXISES < 1)
  #error AXISES must be between 1-3 (inclusive).
#endif

#define STEP_DELAY 50 // How long to wait in between steps
#define DEBUG false  // Whether or not to print debug statements over the serial connection. (This should always be false for production use.)

void setup() {
  Serial.begin(9600);
  pinMode(STEP_PIN_0, OUTPUT);
  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(HOME_PIN_0, INPUT);
  if (AXISES >= 2) {
    pinMode(STEP_PIN_1, OUTPUT);
    pinMode(DIR_PIN_1, OUTPUT);
    pinMode(HOME_PIN_1, INPUT);
  }
  if (AXISES >= 3) {
    pinMode(STEP_PIN_2, OUTPUT);
    pinMode(DIR_PIN_2, OUTPUT);
    pinMode(HOME_PIN_2, INPUT);
  }
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'A') {  // Get number of axises
      Serial.write('0');
      Serial.write(' ');
      Serial.write('0'+AXISES);
    }
    else {
      int axis = (int)(Serial.read()-'0');
      if ((axis >= AXISES) || (axis < 0)) {
        Serial.write('2');
        if (DEBUG) {
          Serial.println(" Invalid axis.");
        }
      }
      if (cmd == '+') { // Move forward
        moveForward(axis, Serial.parseInt());
        Serial.write('0');
        if (DEBUG) {
          Serial.println(" Moved Forward.");
        }
      }
      else if (cmd == '-') { // Move backward
        moveBackward(axis, Serial.parseInt());
        Serial.write('0');
        if (DEBUG) {
          Serial.println(" Moved Backward.");
        }
      }
      else if (cmd == 'H') {  // Get state of home switch
        Serial.write('0');
        Serial.write(' ');
        if (isHome(axis)) {
          Serial.write('T');
        }
        else {
          Serial.write('F');
        }
      }
      else {
        Serial.write('1');
        if (DEBUG) {
          Serial.println(" Unknown Command.");
        }
      }
    }
  }
}

void moveForward(int axis, unsigned long amt) {
  if (axis == 0) {
    digitalWrite(DIR_PIN_0,LOW);
    digitalWrite(STEP_PIN_0,LOW);
    //delay(STEP_DELAY);
    while (amt > 0) {
      digitalWrite(STEP_PIN_0,HIGH);
      delay(STEP_DELAY);
      digitalWrite(STEP_PIN_0,LOW);
      delay(STEP_DELAY);
      amt-= 1;
    }
  }
  else if ((AXISES >= 2) && (axis == 1)) {
    digitalWrite(DIR_PIN_1,LOW);
    digitalWrite(STEP_PIN_1,LOW);
    //delay(STEP_DELAY);
    while (amt > 0) {
      digitalWrite(STEP_PIN_1,HIGH);
      delay(STEP_DELAY);
      digitalWrite(STEP_PIN_1,LOW);
      delay(STEP_DELAY);
      amt-= 1;
    }
  }
  else if ((AXISES >= 3) && (axis == 2)) {
    digitalWrite(DIR_PIN_2,LOW);
    digitalWrite(STEP_PIN_2,LOW);
    //delay(STEP_DELAY);
    while (amt > 0) {
      digitalWrite(STEP_PIN_2,HIGH);
      delay(STEP_DELAY);
      digitalWrite(STEP_PIN_2,LOW);
      delay(STEP_DELAY);
      amt-= 1;
    }
  }
}

void moveBackward(int axis, unsigned long amt) {
  if (axis == 0) {
    digitalWrite(DIR_PIN_0,HIGH);
    digitalWrite(STEP_PIN_0,LOW);
    //delay(STEP_DELAY);
    while (amt > 0) {
      digitalWrite(STEP_PIN_0,HIGH);
      delay(STEP_DELAY);
      digitalWrite(STEP_PIN_0,LOW);
      delay(STEP_DELAY);
      amt-= 1;
    }
  }
  else if ((AXISES >= 2) && (axis == 1)) {
    digitalWrite(DIR_PIN_1,HIGH);
    digitalWrite(STEP_PIN_1,LOW);
    //delay(STEP_DELAY);
    while (amt > 0) {
      digitalWrite(STEP_PIN_1,HIGH);
      delay(STEP_DELAY);
      digitalWrite(STEP_PIN_1,LOW);
      delay(STEP_DELAY);
      amt-= 1;
    }
  }
  else if ((AXISES >= 3) && (axis == 2)) {
    digitalWrite(DIR_PIN_2,HIGH);
    digitalWrite(STEP_PIN_2,LOW);
    //delay(STEP_DELAY);
    while (amt > 0) {
      digitalWrite(STEP_PIN_2,HIGH);
      delay(STEP_DELAY);
      digitalWrite(STEP_PIN_2,LOW);
      delay(STEP_DELAY);
      amt-= 1;
    }
  }
}

bool isHome(int axis) {
  if (axis == 0) {
    return digitalRead(HOME_PIN_0);
  }
  else if ((AXISES >= 2) && (axis == 1)) {
    return digitalRead(HOME_PIN_1);
  }
  else if ((AXISES >= 3) && (axis == 2)) {
    return digitalRead(HOME_PIN_2);
  }
}

