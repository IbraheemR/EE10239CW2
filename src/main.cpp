#include <Arduino.h>

#define TRIMMER_PIN A0

char mode;
bool hasMode = false;

void setupA()
{
  Serial.println("Running A");
}
void setupB()
{
  Serial.println("Running B");
}
void setupC()
{
  Serial.println("Running C");
}
void setupD()
{
  Serial.println("Running D");
}
void setupE()
{
  Serial.println("Running E");
}

void loopA()
{
  int val = analogRead(TRIMMER_PIN);
  float voltage = val / 1023.0 * 5.0;
  Serial.print(val);
  Serial.print("=");
  Serial.print(voltage);
  Serial.println("V");
  delay(100);
}

void loopB()
{
  Serial.print("B");
}

void loopC()
{
  Serial.print("C");
}

void loopD()
{
  Serial.print("D");
}

void loopE()
{
  Serial.print("E");
}

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
    setupA();
    hasMode = true;
  }
  else if (mode == 'B')
  {
    setupB();
    hasMode = true;
  }
  else if (mode == 'C')
  {
    setupC();
    hasMode = true;

  }
  else if (mode == 'D')
  {
    setupD();
    hasMode = true;
  }
  else if (mode == 'E')
  {
    setupE();
    hasMode = true;
  }
}