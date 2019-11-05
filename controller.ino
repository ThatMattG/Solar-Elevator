// ____________________________________________________________
//
// Control system for a solar elevator, or any other device which
// requires a motor to run in one direction at maximum power
// until a button is pressed and then execute a controlled return.
//
// Makes use of a push button, motor, L9110 motor driver and HC-
// SR04 ultra-sonic distance sensor (or compatible alternatives)
// ____________________________________________________________
//
// Written between 27/9/2018 and 6/10/2018
// by ThatMattG (github.com/ThatMattG)
//
// Distance sensor code adapted from:
// https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
//
// Push button code and relevant pin arrangement adapted from:
// https://www.instructables.com/id/Arduino-Button-with-no-resistor/
// ____________________________________________________________

// ____________________________________________________________
//
// Defined values:
// ____________________________________________________________

// The size of an array used to hold distance values
#define DIST_ARR_SIZE 9

// Maximum range of distance sensor. Greater values are unreliable
#define MAX_RANGE 3200

#define NO_COMMAND '\0'
#define UNKNOWN -1

// ____________________________________________________________
//
// Global constants:
// ____________________________________________________________

// External button is connected to pins 8 and GND
const int buttonPin = 8;

// LED on Arduino
const int ledPin = 13;

// Motor pins connect to motor driver
const int motorPin1 = 9;
const int motorPin2 = 10;

// Pins connected to distance sensor
const int distTrigPin = 11;
const int distEchoPin = 12;

// Maximum and increment values for pulse-width modulation
const int pwmMax = 255;
const int pwmIncrement = pwmMax / 5;

// ____________________________________________________________
//
// Global variables:
// ____________________________________________________________

// Distance and time variables are used to calculate speed
long prevDist;
long prevTime;
int prevSpeed;

long currDist = UNKNOWN;
long currTime = UNKNOWN;
int currSpeed= UNKNOWN;

// ascend variable is set to false after reaching top of tether
boolean ascend = true;

// Initialise PWM at 100% power
int pwmSetting = pwmMax;

// currentDirection can be assigned s, f or b.
// Initialise with f to start climbing immediately
char currentDirection = 'f';

// ____________________________________________________________
//
// Subroutine
// Sets button pin for input and the associated LED pin for
// output using global constants.
//
// A push button should be attached to pins 8 and GND, or
// alternatively a button push may be emulated by completing
// the circuit between these two pins
//
// Note: LED will turn on after button is pushed (indicating
// elevator descent).
// This subroutine runs once during set-up.
// ____________________________________________________________

void initialiseButtonPins(){

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  // Announce pins to user
  Serial.print("- Button Pin: ");
  Serial.println(buttonPin);
  Serial.print("- LED Pin: ");
  Serial.println(ledPin);

}

// ____________________________________________________________
//
// Subroutine
// Sets 2 pins for motor output using global constants
// Runs once during set-up.
// ____________________________________________________________

void initialiseMotorPins(){

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // Announce pins to user
  Serial.print("- Motor Pin 1: ");
  Serial.println(motorPin1);
  Serial.print("- Motor Pin 2: ");
  Serial.println(motorPin2);

}

// ____________________________________________________________
//
// Subroutine
// Sets one output and one input pin for use of an ultra-sonic
// distance sensor. Runs once during set-up.
// ____________________________________________________________

void initialiseDistanceSensorPins(){

  pinMode(distTrigPin, OUTPUT);
  pinMode(distEchoPin, INPUT);

  // Announce pins to user
  Serial.print("- Distance Sensor Trigger Pin: ");
  Serial.println(distTrigPin);
  Serial.print("- Distance Sensor Echo Pin: ");
  Serial.println(distEchoPin);

}

// ____________________________________________________________
//
// Function
// Returns one incoming byte if available
// Used primarily for testing, as elevator is autonomous
// ____________________________________________________________

char receiveCommand(){

  // inputByte initialised as NO_COMMAND (ie the null char '\0')
  char inputByte = NO_COMMAND;

  // incomingByte takes the value of one input byte if available
  if (Serial.available()) {
    inputByte = Serial.read();
  }

  return inputByte;

}

// ____________________________________________________________
//
// Function
// Determines if a character is a direction command.
//  - If so, that character is returned.
//  - If not, the function returns the existing currentDirection
// ____________________________________________________________

char updateDirection(char command, char currentDirection){

  // Strings describing the possible directions
  char directionWords[3][16] = {"forwards",
                                "reverse",
                                "stop"};

  int i = 0;
  while (directionWords[i][0] != '\0') {

    // User input is compared with the first letter of each string
    if (directionWords[i][0] == command) {

      // If command is a direction, the direction is printed and returned
      Serial.println(directionWords[i]);
      return command;

    }

    i++;

  }

  return currentDirection;

}

// ____________________________________________________________
//
// Function
// Determines if a character is a speed command.
//  - If so, the  corresponding PWM is returned:
//
//    | Character |  PWM  |
//    |     1     |  51   |
//    |     2     |  102  |
//    |     3     |  153  |
//    |     4     |  204  |
//    |     5     |  255  |
//
//  - If not, the function returns the existing currentSpeed
// ____________________________________________________________

int updatePwm(char command, int pwmSetting){

  int pwmNew;

  // Convert the input into an int
  int commandInt = charToInt(command);

  // Check if input is in the allowed range for PWM commands
  if (commandInt >= 1 && commandInt <= pwmMax / pwmIncrement) {

    pwmNew = pwmIncrement * commandInt;

    // Announce the new speed setting
    Serial.print("Speed updated: ");
    Serial.print(20 * commandInt);
    Serial.println("%");

    return pwmNew;

  }

  // Return existing PWM setting if command is not valid
  return pwmSetting;

}

// ____________________________________________________________
//
// Function
// Converts one char to an int and returns if valid
// ____________________________________________________________

int charToInt(char character) {

  // Integer value is determined by subtracting the ASCII
  // value of the char '0'
  int returnVal = character - '0';

  // Checks if the value is between 0 and 9 inclusive, since
  // that is the maximum valid range for a single char to
  // be converted to an int.
  if (returnVal >=0 && returnVal <=9) {
    return returnVal;
  }

  // returns UNKNOWN (-1) if not a valid integer
  return UNKNOWN;

}

// ____________________________________________________________
//
// Subroutine
// Updates motor pin output using direction and PWM values
//    | Direction |  Pin1  |  Pin2  |
//    |     f     |    0   |   PWM  |
//    |     r     |   PWM  |    0   |
//    |     s     |    0   |    0   |
// ____________________________________________________________

void setMotorPins(int dir, int pwm){

  // Reset both pins and delay 50ms
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  delay(50);

  // Direction will be printed in the relevant if statement below
  Serial.print("Direction: ");

  // Case: forwards
  if (dir == 'f') {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, pwm);
    Serial.println("forwards");
  }

  // Case: reverse
  else if (dir == 'r') {
    analogWrite(motorPin1, pwm);
    analogWrite(motorPin2, 0);
    Serial.println("reverse");
  }

  // Case: stopped
  else if (dir == 's') {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    Serial.println("stopped");
  }

  // Print PWM value
  Serial.print("Power: ");
  Serial.println(pwm);

}

// ____________________________________________________________
//
// Subroutine
// Initiates descent of the elevator.
// ____________________________________________________________

void beginDescent() {

  // Turn off ascent variable and indicate descent by activating LED
  ascend = false;
  digitalWrite(ledPin, HIGH);

  Serial.println("BEGINNING DESCENT");

  // Reverse elevator with maximum power
  currentDirection = 'r';
  pwmSetting = pwmMax;
  setMotorPins(currentDirection, pwmSetting);

}

// ____________________________________________________________
//
// Fills an array of size "repeat" with distance measurements.
// An array is also filled with the time (ms) of each measurement.
// ____________________________________________________________

void getDistances(int repeat, long dists[], long times[]) {

  long timeTaken;
  long distance;
  long currTime;

  // distanceNum is a counter
  int distanceNum = 0;
  while (distanceNum < repeat) {

    // Sensor should receive a short LOW signal first
    digitalWrite(distTrigPin, LOW);
    delayMicroseconds(5);

    // A 10ms HIGH signal triggers the ultra-sonic output
    digitalWrite(distTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(distTrigPin, LOW);

    // Measure time taken for input
    timeTaken = pulseIn(distEchoPin, HIGH);

    // Record the internal clock time
    currTime = millis();

    // Determine distance (mm)
    distance = timeTaken * 10 / 58;

    // Append arrays if distance is within valid range
    if (distance < MAX_RANGE) {
      dists[distanceNum] = distance;
      times[distanceNum] = currTime;

      distanceNum++;
    }
  }

}

// ____________________________________________________________
//
// Function
// Returns the index of the minimum value in an array of longs
// ____________________________________________________________

int getIndexLowest(int length, long arr[]) {

  long lowest = arr[0];
  int indexLowest = 0;

  int i = 1;
  // Iterate through list, maintaining a record of the lowest
  // value and its index (begining with index 0)
  while (i < length) {

    if (arr[i] < lowest) {

      // Lowest value and its index are stored
      lowest = arr[i];
      indexLowest = i;

    }

    i++;
  }

  // Index of the lowest element is returned
  return indexLowest;

}

// ____________________________________________________________
//
// Function
// Calculates speed from 2 pairs of distance and time measurements.
// Return value will be positive in the downward direction as
// this simplifies the logic for speed adjustments.
// Returns UNKNOWN (-1) if a distance or time is invalid.
//
// Known issue: if the elevator is calculated to be ascending at
// 1cm/second (-1), the returned value from this function can be
// interpreted as UNKNOWN. This is minor, since another ~3 speed
// calculations will occur in the next second and the elevator
// should be descending; a true 1cm/sec ascent is unlikely.
// ____________________________________________________________
long calcSpeed(int currDist, int prevDist, long currTime, long prevTime) {

  // Speed is unknown if any of the 4 parameters is unknown
  if (currDist == UNKNOWN || prevDist == UNKNOWN ||
      currTime == UNKNOWN || prevTime == UNKNOWN){
    return UNKNOWN;
  }

  // Speed is unknown if either distance measurement is outside range
  if (currDist > MAX_RANGE || prevDist > MAX_RANGE) {
    return UNKNOWN;
  }

  // Determines distance travelled (>0 if moving towards destination)
  long mmTravelled = prevDist - currDist;
  // Time elapsed in ms
  long timeElapsed = currTime - prevTime;

  // Speed is unknown in the (very rare) case of an overflow
  if (timeElapsed < 0) {
    return UNKNOWN;
  }

  // Calculate speed as distance/time in unit mm/sec.
  // Multiplying by 1000 since timeElapsed is measured in ms.
  long spd = mmTravelled * 1000 / timeElapsed;

  return spd;
}

// ____________________________________________________________
//
// Subroutine
// Adjusts the elevator's descent based on both raw distance
// and the calculated speed.
// While a concise algorithm for speed adjustment would
// be ideal, desired speed is highly variable based on altitude.
// So although this function is long and inelegant, it provides
// a good template for tweaking settings at various heights.
// Aim to condense this code following testing.
// ____________________________________________________________
void adjustDescent(long dist, long spd) {

  // ============================================================

  // If elevator appears to be ascending and is >10cm from base,
  // the motor should be set in reverse at 80% power
  if (spd < 0 && dist > 200){

    Serial.println("Going up? That's not right.");

    // Set direction, power and apply these settings to motor pins
    currentDirection = 'r';
    pwmSetting = pwmIncrement * 4;
    setMotorPins(currentDirection, pwmSetting);

  }

  // ============================================================

  // Case: <8 cm from base
  else if (dist < 80) {

    Serial.println("Elevator has landed!");

    // Set motor to forwards at 20% power (likely to cease movement)
    currentDirection = 'f';
    pwmSetting = pwmIncrement * 1;
    setMotorPins(currentDirection, pwmSetting);

  }

  // ============================================================

  // Case: <50 cm from base
  else if (dist < 500) {

    // Speed >25cm/sec considered very excessive
    if (spd > 250) {

      Serial.println("Dangerous speed! Motor will be placed in forward direction.");

      // Set motor to forwards at 80% power to slow the elevator immediately
      currentDirection = 'f';
      pwmSetting = pwmIncrement * 4;
      setMotorPins(currentDirection, pwmSetting);

    }

    // Speed >15cm/sec considered too fast
    else if (spd > 150) {

      Serial.println("Too fast.");

      // Set motor to forwards at 40% power to slow the elevator
      currentDirection = 'f';
      pwmSetting = pwmIncrement * 2;
      setMotorPins(currentDirection, pwmSetting);

    }

    // Speed <5cm/sec considered too slow
    else if (spd < 50) {

      Serial.println("Too slow.");

      // Set motor to reverse at 60% power to bring elevator down
      currentDirection = 'r';
      pwmSetting = pwmIncrement * 3;
      setMotorPins(currentDirection, pwmSetting);

    }
  }

  // ============================================================

  // Case: <1.5m from base
  else if (dist < 1500) {

    // Speed >50cm/sec considered too fast
    if (spd > 500) {

      Serial.println("Too fast.");

      // Set motor to forwards at 40% power to slow elevator
      currentDirection = 'f';
      pwmSetting = pwmIncrement * 2;
      setMotorPins(currentDirection, pwmSetting);

    }

    // Speed <20cm/sec considered too slow
    else if (spd < 200) {

      Serial.println("Too slow.");

      // Set motor to reverse at 60% power to bring elevator down
      currentDirection = 'r';
      pwmSetting = pwmIncrement * 4;
      setMotorPins(currentDirection, pwmSetting);

    }
  }

  // ============================================================

  // Case: >1.5m from base
  else {

    // If the elevator would reach the base within 5 secs at
    // current speed, it is considered too fast
    if (spd * 5 > currDist) {

      Serial.println("Too fast.");

      // Turn off the motor (elevator should coast to unknown degree)
      currentDirection = 's';
      setMotorPins(currentDirection, pwmSetting);

    }

    // If the elevator would take > 20 secs to reach the base
    // at current speed, it is considered too slow
    else if (spd * 20 < currDist) {

      Serial.println("Too slow.");

      // Set motor to reverse at full power
      currentDirection = 'r';
      pwmSetting = pwmIncrement * 5;
      setMotorPins(currentDirection, pwmSetting);

    }
  }

  // ============================================================

}

// ____________________________________________________________
//
// Subroutine
// Print a summary of the distance to base as well as height
// Distance is additionally graphed with a '#'
// ____________________________________________________________

void printSummary(long dist, int spd) {

  // Convert dist to cm
  dist = dist / 10;
  // Convert spd to cm/sec
  spd = spd / 10;

  // Determine the direction that the elevator travelled
  boolean down = true;
  if (spd < 0) {
    down = false;
    // Set the speed to its absolute value
    spd = - spd;
  }

  // Distance and speed are printed in clear language and simple
  // units with an indent to distinguish from other information
  // (as multiple output lines may appear 3-4 times per second)
  Serial.print("\t\t\t");
  Serial.print(dist);
  Serial.print(" cm from target and travelling at ");
  Serial.print(spd);
  Serial.print(" cm/sec");

  // Print the direction that the elevator is travelling
  if (spd == 0) {
    Serial.println(".");
  }
  else if (down == true) {
    Serial.println(" towards it.");
  }
  else {
    Serial.println(" away from it.");
  }

  if (dist < MAX_RANGE) {

    Serial.print("(#) Distance graph: ");

    int i = 0;

    // Print one space for every 50mm distance
    while (i < dist) {
      Serial.print(" ");
      i += 5;
    }

    Serial.println("#");

  }

}

// ____________________________________________________________
//
// Setup runs initialisation process once at start-up
// ____________________________________________________________

void setup(){

  // Set the rate for data transmission
  Serial.begin(9600);

  Serial.println("Solar Elevator Control System is initialising:");

  // Set button pin and the associated LED pin
  initialiseButtonPins();

  // Set motor pins based on global constants
  initialiseMotorPins();

  // Set distance sensor pins
  initialiseDistanceSensorPins();

  // Inform the user that initialisation is complete
  // Provide summary of valid inputs and elevator path
  Serial.println("Initialisation successfully completed.");
  Serial.println("- Use 'f', 'r' and 's' to control direction.");
  Serial.println("- Use '1', '2', '3', '4' and '5' to control speed.");
  Serial.println("Elevator will ascend until a button push, then return.");

  // Motor pins are set with initial values (ascent at full power)
  setMotorPins(currentDirection, pwmSetting);

}

// ____________________________________________________________
//
// Loop will repeat for the remainder of run-time
// ____________________________________________________________

void loop(){

  // Receive a command if available, otherwise command will be set
  // to NO_COMMAND
  char command = receiveCommand();

  // This code runs if a valid command has been received
  if (command != NO_COMMAND) {

    // If the command is 'f', 'r' or 's', currentDirection will update
    currentDirection = updateDirection(command, currentDirection);

    // If the command is a number 1-5, PWM will be updated
    pwmSetting = updatePwm(command, pwmSetting);

    // Applies direction and PWM settings to motor pins
    setMotorPins(currentDirection, pwmSetting);

  }

  // This code runs if elevator has not reached the top of the tether yet
  else if (ascend == true) {

    if (digitalRead(8) == LOW) {
      Serial.println("Pressed");
      beginDescent();
    }

  }

  // This code runs if elevator has already reached the top of the tether
  else if (ascend == false){

    prevDist = currDist;
    prevTime = currTime;
    prevSpeed = currSpeed;

    long distsArray[DIST_ARR_SIZE];
    long timesArray[DIST_ARR_SIZE];

    getDistances(DIST_ARR_SIZE, distsArray, timesArray);

    // Vastly over-estimated distance measurements are common, while
    // under-estimated measurements are rare.
    // The lowest distance reading in a small array is usually a reliable
    // measure of the distance (more so than the median and easier to obtain).
    // The index of the lowest distance measurement in an array is determined.
    int lowestDistIndex = getIndexLowest(DIST_ARR_SIZE, distsArray);

    // The corresponding distance and time stamps are used for speed calculation.
    currDist = distsArray[lowestDistIndex];
    currTime = timesArray[lowestDistIndex];

    currSpeed = calcSpeed(currDist, prevDist, currTime, prevTime);

    // If the current speed is valid, print a summary of distance/speed
    // and then adjust the motor settings if necessary
    if (currSpeed != UNKNOWN) {
      printSummary(currDist, currSpeed);
      adjustDescent(currDist, currSpeed);
      Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - ");
      Serial.println("");
    }

    // Appropriate delay to allow for a measurable distance to be travelled
    delay(240);
  }

  delay(10);
}
