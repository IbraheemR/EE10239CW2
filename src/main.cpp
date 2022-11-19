#include <Arduino.h>
#include <Encoder.h>

#define TRIMMER_PIN A0

char program;
bool hasProgram = false;

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

#define MOTOR_A 5
#define MOTOR_B 6

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

#define MOTOR_SENSE_A 2
#define MOTOR_SENSE_B 3

Encoder encoder(MOTOR_SENSE_A, MOTOR_SENSE_B);

unsigned long lastTime = 0;
int32_t lastPosition = 0;

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
  int motorDutyCycle = val / 4; // Map max 1023 to max 255

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
  Serial.println("Running E");
}

void loopE()
{
  Serial.print("E");
}

// ---------------------------------

void setup()
{
  Serial.begin(9600);
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

    default:
      break;
    }
    return;
  }

  program = toupper(Serial.read());

  if (program == 'A')
  {
    Serial.println("Progam A:");
    setupA();
    hasProgram = true;
  }
  else if (program == 'B')
  {
    Serial.println("Progam B:");
    setupB();
    hasProgram = true;
  }
  else if (program == 'C')
  {
    Serial.println("Progam C:");
    setupC();
    hasProgram = true;
  }
  else if (program == 'D')
  {
    Serial.println("Progam D:");
    setupD();
    hasProgram = true;
  }
  else if (program == 'E')
  {
    Serial.println("Progam E:");
    setupE();
    hasProgram = true;
  }
}