#include <Motoron.h>

MotoronI2C mc;

const auto vinType = MotoronVinSenseType::MotoronHp;
const uint16_t referenceMv = 5000;
const uint16_t minVinVoltageMv = 6500;

int rightline;
int leftline;

// IR sensor pins
const int CENTER_PIN = 7;
const int LEFT_X_PIN = 1;
const int LEFT_PAR = 2;
const int RIGHT_PAR = 4;
const int RIGHT_X_PIN = 3;

const int RIGHT_LINE = 6;
const int LEFT_LINE  = 5;

// Control pins
const int START_PIN = 12;

// pin state variables 
int centerDetected;
int leftXDetected;
int rightXDetected;
int parallelLeftDetected;
int parallelRightDetected;
int frontLeft;
int frontRight;
int backLeft;
int backRight;

const uint16_t errorMask = 
  (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR) |
  (1 << MOTORON_STATUS_FLAG_CRC_ERROR) |
  (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_NO_POWER_LATCHED) |
  (1 << MOTORON_STATUS_FLAG_RESET) |
  (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT) |
  (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING) |
  (1 << MOTORON_STATUS_FLAG_NO_POWER) |
  (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE);

void checkCommunicationError(uint8_t errorCode)
{
  if (errorCode)
  {
    while (1)
    {
      mc.reset();
      Serial.print(F("Communication error: "));
      Serial.println(errorCode);
      delay(1000);
    }
  }
}

void checkControllerError(uint16_t status)
{
  if (status & errorMask)
  {
    while (1)
    {
      mc.reset();
      Serial.print(F("Controller error: 0x"));
      Serial.println(status, HEX);
      delay(1000);
    }
  }
}

void checkVinVoltage(uint16_t voltageMv)
{
  if (voltageMv < minVinVoltageMv)
  {
    while (1)
    {
      mc.reset();
     Serial.print(F("VIN voltage too low: "));
      Serial.println(voltageMv);
      delay(1000);
    }
  }
}

void checkForProblems() {
  uint16_t status = mc.getStatusFlags();
  checkCommunicationError(mc.getLastError());
  checkControllerError(status);

  checkCommunicationError(mc.getLastError());
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mc.reinitialize();
  mc.clearResetFlag();

  mc.setErrorResponse(MOTORON_ERROR_RESPONSE_BRAKE);
  mc.setErrorMask(errorMask);

  mc.setCommandTimeoutMilliseconds(100);

  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 300);

  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 300);

  while(mc.getMotorDrivingFlag())
    mc.clearMotorFaultUnconditional();

  pinMode(LEFT_LINE, INPUT);
  pinMode(RIGHT_LINE, INPUT);

  pinMode(START_PIN, INPUT);
}

void MoveForwardFULL() {
  mc.setSpeed(1, 800);
  mc.setSpeed(2, 800);
}

void MoveForward() {
  mc.setSpeed(1, 600);
  mc.setSpeed(2, 600);
}

void MoveBackward() {
  mc.setSpeed(1, -800);
  mc.setSpeed(2, -800);
}

void RotateLeft() {
  mc.setSpeed(1, 320);
  mc.setSpeed(2, -320);
}

void RotateRight() {
  mc.setSpeed(1, -320);
  mc.setSpeed(2, 320);
}

void idleMode() {
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
}

void stopMode() {
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
}

void BeginSearch() {
  centerDetected = digitalRead(CENTER_PIN);
  leftXDetected = digitalRead(LEFT_X_PIN);
  rightXDetected = digitalRead(RIGHT_X_PIN);
  rightline = digitalRead(RIGHT_LINE);
  leftline = digitalRead(LEFT_LINE);

  if (centerDetected && leftXDetected && rightXDetected ) {
    // all sensors
    MoveForwardFULL();
  } else if (!centerDetected && leftXDetected && !rightXDetected ) {
    // left
    RotateLeft();
    MoveForward();
  } else if (!centerDetected && !leftXDetected && rightXDetected ) {
    // right
    RotateRight();
    MoveForward();
  } else if (centerDetected && !leftXDetected && !rightXDetected ) {
    // center
    RotateRight();
    MoveForward();
  } else if (centerDetected && leftXDetected && !rightXDetected ) {
    // center left
    RotateRight();
    MoveForward();
  } else if (centerDetected && !leftXDetected && rightXDetected ) {
    // center right
    RotateRight();
    MoveForward();
  }
}

enum State {
  STATE_IDLE,
  STATE_MOVE_BACKWARD,
  STATE_MOVE_FORWARD
};

State currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
const unsigned long backwardDuration = 1000; // 1 second

void updateState() {
  unsigned long currentTime = millis();
  switch (currentState) {
    case STATE_IDLE:
      if (digitalRead(START_PIN) == HIGH) {
        currentState = STATE_MOVE_BACKWARD;
        stateStartTime = currentTime;
        MoveBackward();
      }
      break;
    case STATE_MOVE_BACKWARD:
      if (currentTime - stateStartTime >= backwardDuration) {
        currentState = STATE_MOVE_FORWARD;
        BeginSearch();
      }
      break;
    case STATE_MOVE_FORWARD:
      BeginSearch();
      break;
  }
}

void loop() {
  checkForProblems();
  updateState();
}

