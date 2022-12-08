#include <Arduino.h>
#include <Encoder.h>

#define TRIMMER_PIN A0

#define MOTOR_A 5
#define MOTOR_B 6

// Interrupts set on pins 2 & 3 have the highest frequency and thus accuracy
#define ENCODER_A 2
#define ENCODER_B 3

// Tracks which program is selected on startup
char programSelect;
bool hasProgram = false;

Encoder encoder(ENCODER_A, ENCODER_B);

// Misc. timing vars
unsigned long lastTime = 0;
int32_t lastPosition = 0;

// Control var for speed control
int motorDutyCycle = 0;

// Config for servo controller

#define COUNTS_PER_DEGREE 12.0 * 298.0 / 360.0
// Ramp the motor voltage in region ±30 deg around setpoint
#define RAMP_REGION 30
#define DEAD_REGION 1

#define RAMP_COUNTS RAMP_REGION *COUNTS_PER_DEGREE
#define DEAD_COUNTS DEAD_REGION *COUNTS_PER_DEGREE

// Config for secret test modes

#define NUM_TRIALS 100
#define TRIAL_TIME 3000
#define SETPOINT_A -180
#define SETPOINT_B 180

#define SETPOINT_A_COUNTS SETPOINT_A *COUNTS_PER_DEGREE
#define SETPOINT_B_COUNTS SETPOINT_B *COUNTS_PER_DEGREE

double setpoint = SETPOINT_A_COUNTS;
unsigned long testStartMillis;
int numTests = 0;

//
// Based on the users input, we delegate on one of these setup & loop methods.
//

// ----------------

void setupA()
{
  //  ...
}

void loopA()
{
  int val = analogRead(TRIMMER_PIN);

  double percentage = val / 1023.0 * 100.0; // Resale 0 to 1023 range to 0 to 100%
  double voltage = val / 1023.0 * 5.0;      // Resale 0 to 1023 range to 0 to 5V

  Serial.print(percentage);
  Serial.print("% = ");
  Serial.print(voltage);
  Serial.println("V");

  delay(100);
}

// -----------------

void setupB()
{
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  Serial.println("Press F for FORWARD, B for BACKWARDS or H for HALT");
}

void setMotorState(char motorMode, int speed)
{
  switch (motorMode)
  {
  case 'F':
    analogWrite(MOTOR_A, speed);
    analogWrite(MOTOR_B, 0);
    break;

  case 'B':
    analogWrite(MOTOR_A, 0);
    analogWrite(MOTOR_B, speed);
    break;

  case 'H':
    analogWrite(MOTOR_A, 0);
    analogWrite(MOTOR_B, 0);
    break;

  default:
    break;
  }
}

char motorMode = 'H';

void loopB()
{

  if (Serial.available())
  {
    char c = Serial.read();
    c = toupper(c);
    // Check if it's a valid mode
    if (c == 'F' || c == 'B' || c == 'H')
    {
      motorMode = c;
      // Print the selected mode
      switch (motorMode)
      {
      case 'F':
        Serial.println("FORWARDS");
        break;

      case 'B':
        Serial.println("BACKWARDS");
        break;

      case 'H':
        Serial.println("HALT");
        break;

      default:
        break;
      }
    }
  }

  setMotorState(motorMode, 255);
}

// -----------------

void setupC()
{
  // Setup is identical for this excersize
  setupB();
}

void calculateAndPrintRPM()
{
  int32_t position = encoder.read();
  unsigned long now = millis();

  unsigned long timeDiff = now - lastTime;
  if (timeDiff >= 1000)
  {
    int32_t posDiff = position - lastPosition;
    double rotations = posDiff / COUNTS_PER_DEGREE / 360; // counts -> rotations
    double minutes = timeDiff / 1000.0 / 60.0;            // ms -> mins
    double speed = rotations / minutes;

    // Reset counter vars
    lastPosition = position;
    lastTime = now;

    Serial.print(speed);
    Serial.println("rpm");
  }
}

void loopC()
{
  // Loop is also identical, we just add extra logic to measure motor speed
  loopB();
  calculateAndPrintRPM();
}

// -----------------

void setupD()
{
  setupB();
}

void loopD()
{
  if (Serial.available())
  {
    char c = Serial.read();
    c = toupper(c);
    // Check if it's a valid mode
    if (c == 'F' || c == 'B' || c == 'H')
    {
      motorMode = c;
      // Print the selected mode
      switch (motorMode)
      {
      case 'F':
        Serial.println("FORWARDS");
        break;

      case 'B':
        Serial.println("BACKWARDS");
        break;

      case 'H':
        Serial.println("HALT");
        break;

      default:
        break;
      }
    }
  }

  // # Read trimmer value
  int val = analogRead(TRIMMER_PIN);
  motorDutyCycle = val / 4; // Map max 1023 to max 255

  setMotorState(motorMode, motorDutyCycle);
  calculateAndPrintRPM();
}

// -----------------

void setupE()
{
  // The motor is taken to be in the 0deg position on startup.
  Serial.println("Servo mode active.");

  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
}

int controlStrategy(double setpointCounts)
{
  int32_t position = encoder.read();

  int error = position - setpointCounts;


  double rampedDiff = 0;

  if (error > DEAD_REGION || error < -DEAD_REGION)
  {
    if (error > RAMP_REGION)
    {
      rampedDiff = 255;
    }
    else if (error < -RAMP_REGION)
    {
      rampedDiff = -255;
    }
    else
    {
      rampedDiff = error / RAMP_REGION * 255;
    }
  }

  int dutyCycle = rampedDiff;

  analogWrite(MOTOR_A, max(-dutyCycle, 0));
  analogWrite(MOTOR_B, max(dutyCycle, 0));

  return dutyCycle;
}

void loopE()
{
  // Determine the setpoint from the trimmer.
  int val = analogRead(TRIMMER_PIN);
  double setpointDegs = (val - 512) / 512.0 * 180;          // rescale into -180 to 180 degree range
  double setpointCounts = setpointDegs * COUNTS_PER_DEGREE; // Convert degrees into counts

    // Execute the control strategy
  int pwm = controlStrategy(setpointCounts);

  // -------------------------
  // Print values

  int32_t position = encoder.read();
  unsigned long now = millis();

  unsigned long timeDiff = now - lastTime;
  if (timeDiff >= 1000)
  {
    int32_t posDiff = position - lastPosition;
    double rotations = posDiff / COUNTS_PER_DEGREE / 360; // counts -> rotations
    double minutes = timeDiff / 1000.0 / 60.0;            // ms -> mins
    double speed = rotations / minutes;

    // Reset counter vars
    lastPosition = position;
    lastTime = now;

    Serial.print("rpm=");
    Serial.print(speed);
    Serial.print(" pos=");
    Serial.print(position / COUNTS_PER_DEGREE * 1.46);
    Serial.print("deg trimmer=");
    Serial.print(val/1023.0 * 100.0);
    Serial.print("% PWM=");
    Serial.print(pwm/255.0 * 100.0);
    Serial.println("%");
  }


}

// ---------------------------------

bool rampUp = true;

void setupSpeedTest()
{
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);

  analogWrite(MOTOR_A, 0);
  analogWrite(MOTOR_B, 0);
}

void loopSpeedTest()
{
  int32_t position = encoder.read();
  unsigned long now = millis();

  Serial.print("t=");
  Serial.print(now);
  Serial.print(",duty=");
  Serial.print(motorDutyCycle);
  Serial.print(",pos=");
  Serial.println(position);

  if (now - lastTime > 1000)
  {
    motorDutyCycle += rampUp ? 1 : -1;
    if (rampUp)
    {
      if (motorDutyCycle >= 255)
      {
        rampUp = false;
      }
    }
    else
    {
      if (motorDutyCycle < 0)
      {
        Serial.println("DONE");
        while (1)
          ;
      }
    }

    analogWrite(MOTOR_A, motorDutyCycle);
    lastTime = now;
  }
}

// ---------------------------------

void setupServoStepTest()
{
  setupE();
  testStartMillis = millis();
}

void loopServoStepTest(int totalTrials, bool randomRange)
{
  long testTime = millis() - testStartMillis;

  if (testTime > TRIAL_TIME) // Step 2s into each test
  {
    testStartMillis = millis();
    if (randomRange)
    {
      setpoint = random(SETPOINT_A_COUNTS, SETPOINT_B_COUNTS);
    }
    else
    {
      setpoint = setpoint == SETPOINT_A_COUNTS ? SETPOINT_B_COUNTS : SETPOINT_A_COUNTS;
    }
    numTests++;
  }

  if (numTests > totalTrials)
  {
    Serial.print("DONE.");
    while (1)
      ;
  }

  controlStrategy(setpoint);

  Serial.print("t=");
  Serial.print(millis());
  Serial.print(",setpoint=");
  Serial.print(setpoint);
  Serial.print(",pos=");
  Serial.println(encoder.read());
}

// ---------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.println("Enter Program Code (ABCDE):");
}

void loop()
{
  if (hasProgram)
  {
    switch (programSelect)
    {
    case 'A':
      loopA();
      break;

    case 'B':
      loopB();
      break;

    case 'C':
      loopC();
      break;

    case 'D':
      loopD();
      break;

    case 'E':
      loopE();
      break;

    case '1':
      loopSpeedTest();
      break;

    case '2':
      loopServoStepTest(1, false);
      break;

    case '3':
      loopServoStepTest(NUM_TRIALS, false);
      break;

    case '4':
      loopServoStepTest(NUM_TRIALS, true);
      break;

    default:
      break;
    }
    return;
  }

  programSelect = toupper(Serial.read());

  hasProgram = true;
  switch (programSelect)
  {
  case 'A':
    Serial.println("Progam A:");
    setupA();
    break;

  case 'B':
    Serial.println("Progam B:");
    setupB();
    break;

  case 'C':
    Serial.println("Progam C:");
    setupC();
    break;

  case 'D':
    Serial.println("Progam D:");
    setupD();
    break;

  case 'E':
    Serial.println("Progam E:");
    setupE();
    break;

  case '1':
    Serial.println("Program: Speed Test Mode:");
    setupSpeedTest();
    break;

  case '2':
    Serial.println("Program: Servo Step Response Test:");
    setupServoStepTest();
    break;

  case '3':
    Serial.println("Program: Servo Step Response Test (" + String(NUM_TRIALS) + " Trials):");
    setupServoStepTest();
    break;

  case '4':
    Serial.println("Program: Servo Step Response Test (Random, " + String(NUM_TRIALS) + " Trials):");
    setupServoStepTest();
    break;

  default:
    hasProgram = false;
  }
}