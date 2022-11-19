#include <Arduino.h>
#include <Encoder.h>

#define TRIMMER_PIN A0

char mode;
bool hasMode = false;

void setupA()
{
  Serial.println("Running A");
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

#define MOTOR_A 4
#define MOTOR_B 5

void setupB()
{
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  Serial.println("Press F for FORWARD, B for BACKWARDS or H for HALT");
}

void loopB()
{
  if (Serial.available()) {
    char c = Serial.read();
    c = toupper(c);
    switch (c)
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

  pinMode(LED_BUILTIN, OUTPUT);
}

void loopC()
{
  loopB();

  int32_t position = encoder.read();
  unsigned long now = millis();

  unsigned long timeDiff = now - lastTime;
  if (timeDiff >= 1000) {
    int32_t posDiff = position - lastPosition;
    float rotations = posDiff / (12.0 * 298.0);
    float minutes = timeDiff / 1000.0 / 60.0;
    float speed = - rotations / minutes;

    lastPosition = position;
    lastTime = now;

    Serial.print(speed);
    Serial.println("rpm");
  }
}

// -----------------

void setupD()
{
  Serial.println("Running D");
}

void loopD()
{
  Serial.print("D");
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
  if (hasMode)
  {
    switch (mode)
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

  mode = toupper(Serial.read());

  if (mode == 'A')
  {
    Serial.println("Progam A:");
    setupA();
    hasMode = true;
  }
  else if (mode == 'B')
  {
    Serial.println("Progam B:");
    setupB();
    hasMode = true;
  }
  else if (mode == 'C')
  {
    Serial.println("Progam C:");
    setupC();
    hasMode = true;
  }
  else if (mode == 'D')
  {
    Serial.println("Progam D:");
    setupD();
    hasMode = true;
  }
  else if (mode == 'E')
  {
    Serial.println("Progam E:");
    setupE();
    hasMode = true;
  }
}