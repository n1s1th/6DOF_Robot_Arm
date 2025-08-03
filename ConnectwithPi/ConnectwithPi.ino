// === LIBRARIES ===
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <math.h>

// === PCA9685 SERVO DRIVER ===
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// === SERVO CONFIGURATION ===
#define SERVO_MIN 150
#define SERVO_MID 375
#define SERVO_MAX 600

// Servo mapping (pwm channel)
#define M1_GRIPPER     6
#define M2_WRIST       1
#define M3_WRIST_ROT   2
#define M4_ELBOW       3
#define M5_SHOULDER    4
#define M6_BASE        5

// === LIMIT SWITCHES ===
const int limitHomePin = 4;
const int limitEndPin = 5;

// === STEPPER MOTOR (X AXIS) ===
const int stepPin = 3;
const int dirPin = 2;
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// === ARM SEGMENT LENGTHS === (Placeholder values)
double L1 = 15.0; // Shoulder to Elbow (cm)
double L2 = 15.0; // Elbow to Wrist (cm)
double L3 = 5.0;  // Wrist to Gripper (cm)

// === GLOBAL STATE ===
double angle_shoulder = 90;
double angle_elbow = 90;
double angle_wrist = 90;
double angle_base = 90;
int gripper_open = 170;
int gripper_closed = 125;

// === COMMUNICATION ===
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

// Format: <x,y,z,bool_move,bool_open,delayms,type>
double XYZ_next[7] = {0};

// === FUNCTION DECLARATIONS ===
void moveToPosition(double x, double y, double z, bool liftMotion);
void getAnglesFromYZ(double y, double z);
void moveServo(int ch, int angle);
void recvWithStartEndMarkers();
void parseData();

// === SETUP ===
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(60);

  pinMode(limitHomePin, INPUT_PULLUP);
  pinMode(limitEndPin, INPUT_PULLUP);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Set default position
  moveServo(M1_GRIPPER, gripper_open);
  moveServo(M2_WRIST, 90);
  moveServo(M3_WRIST_ROT, 90);
  moveServo(M4_ELBOW, 90);
  moveServo(M5_SHOULDER, 90);
  moveServo(M6_BASE, 90);

  Serial.println("Ready");
}

// === MAIN LOOP ===
void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    parseData();
    moveToPosition(XYZ_next[0], XYZ_next[1], XYZ_next[2], XYZ_next[3]);
    moveServo(M1_GRIPPER, XYZ_next[4] ? gripper_open : gripper_closed);
    delay(XYZ_next[5]);
    newData = false;
    Serial.println("done");
  }

  if (digitalRead(limitEndPin) == LOW) {
    Serial.println("Emergency Stop Triggered!");
    while (1); // Stop all movement
  }
}

// === IK Y-Z Calculation ===
void getAnglesFromYZ(double y, double z) {
  double H = sqrt(y*y + z*z);
  double angle_elb = acos((L1*L1 + L2*L2 - H*H) / (2 * L1 * L2));
  double angle_shld = atan2(z, y) - acos((H*H + L1*L1 - L2*L2) / (2 * L1 * H));

  angle_elbow = angle_elb * 180.0 / M_PI;
  angle_shoulder = angle_shld * 180.0 / M_PI;

  // Convert to servo-friendly angles
  angle_elbow = 180 - angle_elbow; // adjust for servo direction
  angle_shoulder = angle_shoulder; // already in servo space
}

// === SERVO MOVE ===
void moveServo(int ch, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(ch, 0, pulse);
}

// === POSITION COORDINATE MOVEMENT ===
void moveToPosition(double x, double y, double z, bool liftMotion) {
  getAnglesFromYZ(y, z);
  moveServo(M5_SHOULDER, angle_shoulder);
  delay(300);
  moveServo(M4_ELBOW, angle_elbow);
  delay(300);

  // X axis (stepper) movement
  long steps = x * 50; // adjust STEPS_PER_CM based on your setup
  stepper.moveTo(steps);
  stepper.runToPosition();

  // Optional: Add base rotation logic based on X here
  // Optional: Add wrist orientation based on task type (XYZ_next[6])
}

// === COMMUNICATION ===
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) ndx = numChars - 1;
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  char * strtokIndx;
  strtokIndx = strtok(receivedChars, ","); XYZ_next[0] = atof(strtokIndx);
  strtokIndx = strtok(NULL, ","); XYZ_next[1] = atof(strtokIndx);
  strtokIndx = strtok(NULL, ","); XYZ_next[2] = atof(strtokIndx);
  strtokIndx = strtok(NULL, ","); XYZ_next[3] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); XYZ_next[4] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); XYZ_next[5] = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); XYZ_next[6] = atoi(strtokIndx);
}


//This code was generated with the assistance of ChatGPT.