#include <AccelStepper.h>
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

AccelStepper xStepper(AccelStepper::DRIVER, 23, 25);
AccelStepper yStepper(AccelStepper::DRIVER, 29, 27);
AccelStepper zStepper1(AccelStepper::DRIVER, 49, 47);
AccelStepper zStepper2(AccelStepper::DRIVER, 53, 51);

const int spindleEnable = 4;
const int spindleIn1 = 3;
const int spindleIn2 = 2;

int prevSpindleSpeed = 0;

int currentMessage = -1;
int targetBytes = 4;

float receivedFloat;
float myFloat = 1;
byte byteBuffer[4];

// Sets the spindle speed
void setSpindleSpeed(float speed)
{
  speed *= 255.0/100.0;
  for (int i = 0; i<=1000; i++)
  {
    digitalWrite(spindleIn1, map(i, 0, 1000, prevSpindleSpeed, speed) < 0);
    digitalWrite(spindleIn2, map(i, 0, 1000, prevSpindleSpeed, speed) > 0);
    analogWrite(spindleEnable, abs(map(i, 0, 1000, prevSpindleSpeed, speed)));
    delay(1);
  }
  prevSpindleSpeed = speed;
}

// Converts steps to mm
float stepsToMm(float steps)
{
  return steps/100.0;
}

// Converts mm to steps the stepper has to travel
long mmToSteps(float mm)
{
  return (long)(mm*100.0);
}

// Converts mm/min to steps/seconds
float mmMinToStepSeconds(float speed)
{
  return mmToSteps(speed)/60.0;
}

const long stepperMaxSpeed = mmMinToStepSeconds(1000);
const long stepperAccel = mmToSteps(3600000/3600);

void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(1);

  pinMode(spindleIn1, OUTPUT);
  pinMode(spindleIn2, OUTPUT);
  pinMode(spindleEnable, OUTPUT);

  // Sets up steppers
  xStepper.setMaxSpeed(stepperMaxSpeed);
  yStepper.setMaxSpeed(stepperMaxSpeed);
  zStepper1.setMaxSpeed(stepperMaxSpeed);
  zStepper2.setMaxSpeed(stepperMaxSpeed);

  xStepper.setMinPulseWidth(0);
  yStepper.setMinPulseWidth(0);
  zStepper1.setMinPulseWidth(0);
  zStepper2.setMinPulseWidth(0);
  
  // Sets up stepper acceleration
  xStepper.setAcceleration(stepperAccel);
  yStepper.setAcceleration(stepperAccel);
  zStepper1.setAcceleration(stepperAccel);
  zStepper2.setAcceleration(stepperAccel);

  // Sets current position to 0
  xStepper.setCurrentPosition(0);
  yStepper.setCurrentPosition(0);
  zStepper1.setCurrentPosition(0);
  zStepper2.setCurrentPosition(0);

  // Tells computer that it is ready
  Serial.println("CNC Ready");
}

void loop() {
  // Checks if serial monitor is available
  if (Serial.available() >= targetBytes)
  {
    if (currentMessage == -1)
    {
      // Read first 4 and change current message and target bytes
      for (int i = 0; i < 4; i++) {
        byteBuffer[i] = Serial.read();
      }
      memcpy(&receivedFloat, byteBuffer, sizeof(receivedFloat));
      currentMessage = (int) receivedFloat;

      if (currentMessage == 0)
      {
        xStepper.setCurrentPosition(0);
        yStepper.setCurrentPosition(0);
        zStepper1.setCurrentPosition(0);
        zStepper2.setCurrentPosition(0);
        myFloat = 1;
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));
        currentMessage = -1;
      }
      else if (currentMessage == 1)
      {
        myFloat = stepsToMm(xStepper.currentPosition());
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));
        myFloat = stepsToMm(yStepper.currentPosition());
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));
        myFloat = stepsToMm(zStepper1.currentPosition());
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));
        currentMessage = -1;
      }
      else if (currentMessage == 2)
      {
        targetBytes = 4;
      }
      else if (currentMessage == 3)
      {
        targetBytes = 4;
      }
      else if (currentMessage == 4)
      {
        targetBytes = 4;
      }
      else if (currentMessage == 5)
      {
        targetBytes = 4;
      }
      else if (currentMessage == 6)
      {
        targetBytes = 12;
      }
    }
    else
    {
      if (currentMessage == 2)
      {
        for (int i = 0; i < 4; i++) {
          byteBuffer[i] = Serial.read();
        }
        memcpy(&receivedFloat, byteBuffer, sizeof(receivedFloat));
        setSpindleSpeed(receivedFloat);
        myFloat = 1;
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));

        currentMessage = -1;
        targetBytes = 4;
      }
      else if (currentMessage == 3)
      {
        for (int i = 0; i < 4; i++) {
          byteBuffer[i] = Serial.read();
        }
        memcpy(&receivedFloat, byteBuffer, sizeof(receivedFloat));
        yStepper.setMaxSpeed(stepperMaxSpeed);
        yStepper.runToNewPosition(yStepper.currentPosition()+mmToSteps(receivedFloat));
        myFloat = 1;
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));

        currentMessage = -1;
        targetBytes = 4;
      }
      else if (currentMessage == 4)
      {
        for (int i = 0; i < 4; i++) {
          byteBuffer[i] = Serial.read();
        }
        memcpy(&receivedFloat, byteBuffer, sizeof(receivedFloat));
        zStepper1.setMaxSpeed(stepperMaxSpeed);
        zStepper2.setMaxSpeed(stepperMaxSpeed);
        zStepper1.move(mmToSteps(receivedFloat));
        zStepper2.move(mmToSteps(receivedFloat));
        while (abs(zStepper1.distanceToGo()) > 0 || abs(zStepper2.distanceToGo()) > 0)
        {
          zStepper1.run();
          zStepper2.run();
        }
        zStepper1.setSpeed(0);
        zStepper2.setSpeed(0);
        myFloat = 1;
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));

        currentMessage = -1;
        targetBytes = 4;
      }
      else if (currentMessage == 5)
      {
        for (int i = 0; i < 4; i++) {
          byteBuffer[i] = Serial.read();
        }
        memcpy(&receivedFloat, byteBuffer, sizeof(receivedFloat));
        xStepper.setMaxSpeed(stepperMaxSpeed);
        xStepper.runToNewPosition(xStepper.currentPosition()+mmToSteps(receivedFloat));
        myFloat = 1;
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));

        currentMessage = -1;
        targetBytes = 4;
      }
      else if (currentMessage == 6)
      {
        for (int i = 0; i < 4; i++) {
          byteBuffer[i] = Serial.read();
        }
        memcpy(&receivedFloat, byteBuffer, sizeof(receivedFloat));
        xStepper.setSpeed(mmMinToStepSeconds(receivedFloat));

        for (int i = 0; i < 4; i++) {
          byteBuffer[i] = Serial.read();
        }
        memcpy(&receivedFloat, byteBuffer, sizeof(receivedFloat));
        yStepper.setSpeed(mmMinToStepSeconds(receivedFloat));

        for (int i = 0; i < 4; i++) {
          byteBuffer[i] = Serial.read();
        }
        memcpy(&receivedFloat, byteBuffer, sizeof(receivedFloat));
        zStepper1.setSpeed(mmMinToStepSeconds(receivedFloat));
        zStepper2.setSpeed(mmMinToStepSeconds(receivedFloat));

        myFloat = 1;
        Serial.write((uint8_t*)&myFloat, sizeof(myFloat));

        currentMessage = -1;
        targetBytes = 4;
      }
    }
  }

  xStepper.runSpeed();
  yStepper.runSpeed();
  zStepper1.runSpeed();
  zStepper2.runSpeed();
}
