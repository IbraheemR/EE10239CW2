#include <Arduino.h>
#include <Encoder.h>

#define TRIMMER_PIN A0

#define MOTOR_A 5
#define MOTOR_B 6

#define MOTOR_SENSE_A 2
#define MOTOR_SENSE_B 3

char program;
bool hasProgram = false;

Encoder encoder(MOTOR_SENSE_A, MOTOR_SENSE_B);

unsigned long lastTime = 0;
int32_t lastPosition = 0;

int motorDutyCycle = 0;

void setupA()
{
  //  ...
}

void loopA()
{
  int val = analogRead(TRIMMER_PIN);
  float voltage = val / 1023.0 * 5.0;
  Serial.print(val / 1023.0 * 100.0);
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

char motorMode = 'H';

void loopB()
{
  if (Serial.available())
  {
    char c = Serial.read();
    c = toupper(c);
    if (c == 'F' || c == 'B' || c == 'H')
    {
      motorMode = c;
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

  switch (motorMode)
  {
  case 'F':
    digitalWrite(MOTOR_A, HIGH);
    digitalWrite(MOTOR_B, LOW);
    Serial.println("FORWARDS");
    break;

  case 'B':
    digitalWrite(MOTOR_A, LOW);
    digitalWrite(MOTOR_B, HIGH);
    Serial.println("BACKWARDS");
    break;

  case 'H':
    digitalWrite(MOTOR_A, LOW);
    digitalWrite(MOTOR_B, LOW);
    Serial.println("HALT");
    break;

  default:
    break;
  }
}

// -----------------

void setupC()
{
  setupB();
}

void loopC()
{
  loopB();

  int32_t position = encoder.read();
  unsigned long now = millis();

  unsigned long timeDiff = now - lastTime;
  if (timeDiff >= 1000)
  {
    int32_t posDiff = position - lastPosition;
    float rotations = posDiff / (12.0 * 298.0);
    float minutes = timeDiff / 1000.0 / 60.0;
    float speed = -rotations / minutes;

    lastPosition = position;
    lastTime = now;

    Serial.print(speed);
    Serial.println("rpm");
  }
}

// -----------------

void setupD()
{
  setupB();
}

void loopD()
{
  // # Read trimmer value
  int val = analogRead(TRIMMER_PIN);
  motorDutyCycle = val / 4; // Map max 1023 to max 255

  // set motor speed to trimmer value
  if (Serial.available())
  {
    char c = Serial.read();
    c = toupper(c);
    if (c == 'F' || c == 'B' || c == 'H')
    {
      motorMode = c;
    }
  }

  switch (motorMode)
  {
  case 'F':
    analogWrite(MOTOR_A, motorDutyCycle);
    analogWrite(MOTOR_B, 0);
    break;

  case 'B':
    analogWrite(MOTOR_A, 0);
    analogWrite(MOTOR_B, motorDutyCycle);
    break;

  case 'H':
    analogWrite(MOTOR_A, 0);
    analogWrite(MOTOR_B, 0);
    break;

  default:
    break;
  }

  // read rpm back in
  int32_t position = encoder.read();
  unsigned long now = millis();

  unsigned long timeDiff = now - lastTime;
  if (timeDiff >= 1000)
  {
    int32_t posDiff = position - lastPosition;
    float rotations = posDiff / (12.0 * 298.0);
    float minutes = timeDiff / 1000.0 / 60.0;
    float speed = -rotations / minutes;

    lastPosition = position;
    lastTime = now;

    Serial.print(val / 1023.0 * 100.0);
    Serial.print("% -> ");

    Serial.print(speed);
    Serial.print("rpm [");
    if (motorMode == 'H')
    {
      Serial.print("OFF]");
    }
    else
    {
      Serial.print(speed < 0 ? "C" : "");
      Serial.print("CW]");
    }

    Serial.println();
  }
}

// -----------------

void setupE()
{
  // The motor is taken to be in the 0deg position on startup.
  Serial.println("Servo mode active.");

  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
}

#define COUNTS_PER_DEGREE 12.0 * 298.0 / 360.0
// Ramp the motor voltage in region ±30 deg around setpoint
#define RAMP_REGION 30
#define DEAD_REGION 1

#define RAMP_COUNTS RAMP_REGION * COUNTS_PER_DEGREE
#define DEAD_COUNTS DEAD_REGION * COUNTS_PER_DEGREE

void controlStrategy(float setpointCounts)
{
  int32_t position = encoder.read();

  int diff = position - setpointCounts;

  float rampedDiff = 0;

  if (diff > DEAD_REGION || diff < -DEAD_REGION)
  {
    if (diff > RAMP_REGION)
    {
      rampedDiff = 255;
    }
    else if (diff < -RAMP_REGION)
    {
      rampedDiff = -255;
    }
    else
    {
      rampedDiff = diff / RAMP_REGION * 255;
    }
  }

  int dutyCycle = rampedDiff;

  analogWrite(MOTOR_A, max(-dutyCycle, 0));
  analogWrite(MOTOR_B, max(dutyCycle, 0));
}

void loopE()
{
  // Determine the setpoint from the trimmer.
  int val = analogRead(TRIMMER_PIN);
  float setpointDegs = (val - 512) / 512.0 * 180;          // rescale into -180 to 180 degree range
  float setpointCounts = setpointDegs * COUNTS_PER_DEGREE; // Convert degrees into counts

  controlStrategy(setpointCounts);
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

#define NUM_TRIALS 100
#define TRIAL_TIME 3000
#define SETPOINT_A -180
#define SETPOINT_B 180

#define SETPOINT_A_COUNTS SETPOINT_A * COUNTS_PER_DEGREE
#define SETPOINT_B_COUNTS SETPOINT_B * COUNTS_PER_DEGREE

float setpoint = SETPOINT_A_COUNTS;
unsigned long testStartMillis;
int numTests = 0;

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
    if (randomRange) {
      setpoint = random(SETPOINT_A_COUNTS, SETPOINT_B_COUNTS);
    } else {
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
    switch (program)
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

  program = toupper(Serial.read());

  hasProgram = true;
  switch (program)
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