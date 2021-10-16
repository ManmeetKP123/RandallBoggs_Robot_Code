#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SENSOR0 PA4 //most left
#define SENSOR1 PA5 //left
#define SENSOR2 PA6 //middle
#define SENSOR3 PA7 //right
#define SENSOR4 PB0 //most right

#define POPSTICK_PIN PB_9
#define ARM_PIN PA_8
#define RAMP_PIN PA_9
#define PISTON_PIN PA_10
#define IR_SENSOR PB1

#define MAX_PWM 2500
#define MAX_ANGLE 180
#define MIN_PWM 480
#define MIN_ANGLE 0

#define IR_THRESHOLD 350

//#define KP_POT PB1

#define MOTOR_LEFT_1 PA_0
#define MOTOR_LEFT_2 PA_1
#define MOTOR_RIGHT_1 PA_2
#define MOTOR_RIGHT_2 PA_3

#define MOTORFREQ 100
#define MAX_SPEED 4095
#define MIN_SPEED_FORWARD 1000
#define MIN_SPEED_REVERSE 800
#define TURN_LEFT_MAX 3
#define TURN_RIGHT_MAX 3
#define TURN_SPEED_LEFT 2000
#define TURN_SPEED_RIGHT 2000
#define NORMAL_SPEED 1300
#define SILO_SPEED 950

#define LEFT_ADJUSTMENT 180
#define RIGHT_ADJUSTMENT 0

#define NUM_SILO_HOLES 5

unsigned int reflectance_threshold = 500;

/*https://www.instructables.com/Line-Follower-Robot-PID-Control-Android-Setup/*/
std::array<int, 5> sensorValues {0, 0, 0, 0, 0};

std::array<int, 5> mostLeft {0, 0, 0, 0, 1};
std::array<int, 5> moreLeft {0, 0, 0, 1, 1};
std::array<int, 5> left {0, 0, 0, 1, 0};
std::array<int, 5> bitLeft {0, 0, 1, 1, 0};
std::array<int, 5> center1 {0, 0, 1, 0, 0};
std::array<int, 5> center2 {0, 1, 1, 1, 0};
std::array<int, 5> bitRight {0, 1, 1, 0, 0};
std::array<int, 5> right {0, 1, 0, 0, 0};
std::array<int, 5> moreRight {1, 1, 0, 0, 0};
std::array<int, 5> mostRight {1, 0, 0, 0, 0};

std::array<int, 5> offPath {0, 0, 0, 0, 0};
std::array<int, 5> stopTape {1, 1, 1, 1, 1};

std::array<int, 5> turnRightTape1 {1, 0, 1, 0, 0};
std::array<int, 5> turnRightTape2 {1, 0, 1, 1, 0};
std::array<int, 5> turnRightTape3 {1, 0, 0, 1, 0};

std::array<int, 5> turnLeftTape1 {0, 0, 1, 0, 1};
std::array<int, 5> turnLeftTape2 {0, 1, 1, 0, 1};
std::array<int, 5> turnLeftTape3 {0, 1, 0, 0, 1};

std::array<int, 5> siloStart {1,0,1,0,1};

#define CANS_TO_GET 3
volatile int cansGot = 0;
bool startedRun = false; 

volatile double error = 0;
volatile int P = 0;
volatile int I = 0;
volatile int D = 0;
volatile int PIDvalue = 0;
volatile int previousError = 0;
volatile double Kp = 510;//520;
volatile double Ki = 0;//0.0029;
volatile double Kd = 7000;//3500;

volatile bool atStartOfSilo = false;
volatile bool stop = false;
volatile int allBlack = 0;
bool reachedSilo = false;
volatile bool canDetected = false;

int holesVisited = 0;
int initialSpeed = 1300;

int closedArmAngle = 25;
int openArmAngle = 165;
int closedPistonAngle = 0;
int openPistonAngle = 180;
int downRampAngle = 150;
int upRampAngle = 1;
int openFlippyArm = 145;
int closedFlippyArm = 25;
int midPopAngle = 30;

double slope = (MAX_PWM - MIN_PWM) / (MAX_ANGLE - MIN_ANGLE);

int getPWM(int angle) {
    int pwm = (int)(slope * angle + MIN_PWM);

    if (pwm < MIN_PWM) {
        return MIN_PWM;
    }
    else if (pwm > MAX_PWM) {
        return MAX_PWM;
    }

    return pwm;
}

bool checkSensors(std::array<int, 5> arrToCheck) {
    return sensorValues == arrToCheck;
}

void moveServo(PinName pin, int angle) {
    pwm_start(pin, 50, getPWM(angle), TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

// void readPotentiometer() {
//   Kd = analogRead(KP_POT) * 20; //max is 1023, so divide by constant to get range that you want
//   display.setCursor(0,56);
//   display.print("Kd: ");
//   display.print(Kd, 10);
//   display.display();
// }

void goStraight() {
    pwm_start(MOTOR_LEFT_1, MOTORFREQ, initialSpeed + LEFT_ADJUSTMENT, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_RIGHT_1, MOTORFREQ, initialSpeed + RIGHT_ADJUSTMENT, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
}

void moveServos() {
    moveServo(POPSTICK_PIN, closedFlippyArm);
    delay(1000);
    moveServo(POPSTICK_PIN, openFlippyArm);
    delay(700);
    moveServo(ARM_PIN, closedArmAngle);
    delay(1000);
    moveServo(ARM_PIN, openArmAngle);
    delay(1000);
    moveServo(RAMP_PIN, upRampAngle);
    delay(1500);
    moveServo(RAMP_PIN, downRampAngle);
    delay(1000);
}

void stopRobot() {
    pwm_start(MOTOR_LEFT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_RIGHT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
}

void readSensors() {
  //value of 1 when there is black tape. Somehow black tape reflects more than the ground, so inverse of what is expected
  sensorValues[0] = (analogRead(SENSOR0) > reflectance_threshold) ? 1 : 0;
  sensorValues[1] = (analogRead(SENSOR1) > reflectance_threshold) ? 1 : 0;
  sensorValues[2] = (analogRead(SENSOR2) > reflectance_threshold) ? 1 : 0;
  sensorValues[3] = (analogRead(SENSOR3) > reflectance_threshold) ? 1 : 0;
  sensorValues[4] = (analogRead(SENSOR4) > reflectance_threshold) ? 1 : 0;

}

void pickUpCan() {
    stopRobot();
    delay(100);
    moveServos();
    //goStraight();
    delay(100);
}

void IR_interrupt() {
  canDetected = true;
}

void declareVariables() {
  atStartOfSilo = false;
  stop = false;
  holesVisited = 0;
  initialSpeed = 1300;
  startedRun = false; 
}

void setup() {
  declareVariables();
  pinMode(SENSOR0, INPUT);
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);
  pinMode(SENSOR4, INPUT);

  //pinMode(KP_POT, INPUT_PULLUP);
  
  pinMode(MOTOR_LEFT_1, OUTPUT);
  pinMode(MOTOR_LEFT_2, OUTPUT);
  pinMode(MOTOR_RIGHT_1, OUTPUT);
  pinMode(MOTOR_RIGHT_2, OUTPUT);
  pinMode(IR_SENSOR, INPUT_PULLUP);
  pinMode(RAMP_PIN, OUTPUT);
  pinMode(ARM_PIN, OUTPUT);
  pinMode(PISTON_PIN, OUTPUT);
  pinMode(POPSTICK_PIN, OUTPUT);
  
  readSensors();
  if (checkSensors(stopTape)) {
    delay(12000);
  }

  moveServo(ARM_PIN, openArmAngle);
  moveServo(PISTON_PIN, openPistonAngle);
  delay(700);
  moveServo(POPSTICK_PIN, openFlippyArm);
  /*  END DISPLAY SETUP */

//   moveServo(ARM_PIN, openArmAngle);
//   moveServo(RAMP_PIN, downRampAngle);
//   moveServo(PISTON_PIN, openPistonAngle);
//   moveServo(POPSTICK_PIN, openPopAngle);
//   delay(2000);
  //readPotentiometer();

  //attachInterrupt(IR_SENSOR, IR_interrupt, FALLING);
  //goStraight();
}

void pushCan() {
    moveServo(PISTON_PIN, closedPistonAngle);
    delay(1000);
    moveServo(PISTON_PIN, openPistonAngle);
    delay(1200);
}

void updateError() {
  previousError = error;
  if (checkSensors(mostLeft)) { //black tape on right side, so robot is too left
    error = 4;
  } else if (checkSensors(moreLeft)) {
    error = 3;
  } else if (checkSensors(left)) {
    error = 2;
  } else if (checkSensors(bitLeft)) {
    error = 1;
  } else if (checkSensors(center1) || checkSensors(center2)) {
    error = 0;
  } else if (checkSensors(bitRight)) {
    error = -3;
  } else if (checkSensors(right)) {
    error = -5;
  } else if (checkSensors(moreRight)) {
    error = -6;
  } else if (checkSensors(mostRight)) {
    error = -8;//-8
  } else if (checkSensors(offPath)) {
    if (previousError > 0) {
      error = 5;
    }
    if (previousError <= 0) {
      error = -9;//-9
    }
  } else if (checkSensors(stopTape)) {
    stop = true;
    allBlack++;
  } else if (checkSensors(siloStart)) {
    reachedSilo = true;
  }

  if (atStartOfSilo && holesVisited > 0 && error <= -4) {
    error = -0.5;
  }
}

void calculatePID() {
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (int)((Kp*P) + (Ki*I) + (Kd*D));
}

int keepInSpeedBounds(int speed) {
  if (speed > MAX_SPEED) {
    speed = MAX_SPEED;
  } else if (speed < -MAX_SPEED) {
    speed = -MAX_SPEED;
  } else if (speed >= 0 && speed < MIN_SPEED_FORWARD) {
    speed = MIN_SPEED_FORWARD;
  } else if (speed < 0 && speed > -MIN_SPEED_REVERSE) {
    speed = -MIN_SPEED_REVERSE;
  }

  return speed;
}

void adjustMotors() {
  int left_speed = initialSpeed;
  int right_speed = initialSpeed;

  if (PIDvalue < 0) { //black tape is on the left side, so left wheel needs to turn slower and right wheel faster
    left_speed = left_speed + PIDvalue;
    right_speed = right_speed - PIDvalue;
  }
  else if (PIDvalue > 0) { //black tape is on the right side, so right wheel needs to turn slower
    right_speed = right_speed - PIDvalue;
    left_speed = left_speed + PIDvalue;
  }

  //add adjustments
  left_speed = left_speed >=0 ? left_speed + LEFT_ADJUSTMENT : left_speed - LEFT_ADJUSTMENT;
  right_speed = right_speed >= 0 ? right_speed + RIGHT_ADJUSTMENT : right_speed - RIGHT_ADJUSTMENT;

  left_speed = keepInSpeedBounds(left_speed);
  right_speed = keepInSpeedBounds(right_speed);

  if (left_speed >= 0) {
    pwm_start(MOTOR_LEFT_1, MOTORFREQ, left_speed, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
  } else if (left_speed < 0) {
    pwm_start(MOTOR_LEFT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_LEFT_2, MOTORFREQ, -left_speed, RESOLUTION_12B_COMPARE_FORMAT);
  }
  
  if (right_speed >= 0) {
    pwm_start(MOTOR_RIGHT_1, MOTORFREQ, right_speed, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
  } else {
    pwm_start(MOTOR_RIGHT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_RIGHT_2, MOTORFREQ, -right_speed, RESOLUTION_12B_COMPARE_FORMAT);
  }
}

void checkAndPickUpCan() {
  if (analogRead(IR_SENSOR) < reflectance_threshold && !atStartOfSilo) {
      pickUpCan();
      cansGot++;

    if (cansGot == CANS_TO_GET) {
      moveServo(POPSTICK_PIN, closedFlippyArm);
    }

  }
}

int t1 = 0;

void checkAndAdjustSpeedForSilo() {
  if (checkSensors(stopTape)) {
    if (!atStartOfSilo && (millis() - t1 > 400)) {
      t1 = millis();
      moveServo(POPSTICK_PIN, midPopAngle);
      atStartOfSilo = true;
      initialSpeed = SILO_SPEED;
    }
    else if (holesVisited < NUM_SILO_HOLES) {
      if (((holesVisited == 0) && (millis() - t1 > 400)) ||
      ((holesVisited > 0) && (millis() - t1 > 400))) {
        holesVisited++;
        stopRobot();
        if (holesVisited != 1) {
          pushCan();  
        } else {
          delay(1000);
        }
        t1 = millis();
      }

      if (holesVisited == NUM_SILO_HOLES) {
        holesVisited = 0;
        moveServo(POPSTICK_PIN, openFlippyArm);
        atStartOfSilo = false;
        initialSpeed = NORMAL_SPEED;
        pwm_start(MOTOR_LEFT_1, MOTORFREQ, initialSpeed + LEFT_ADJUSTMENT, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(MOTOR_RIGHT_1, MOTORFREQ, initialSpeed + RIGHT_ADJUSTMENT, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
        delay(200);
        t1 = millis();
      }
    }
  }
}

void intervalChecks() {
  checkAndAdjustSpeedForSilo();
  if (cansGot < CANS_TO_GET) {
    checkAndPickUpCan();
  }
}

int count = 0;

void loop() {
    readSensors();
    if (!startedRun) {
        startedRun = true;
        if (checkSensors(offPath)) {
            pwm_start(MOTOR_LEFT_1, MOTORFREQ, initialSpeed + LEFT_ADJUSTMENT - 0, RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(MOTOR_RIGHT_1, MOTORFREQ, initialSpeed + RIGHT_ADJUSTMENT + 200, RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
            delay(200);
        }
    }
    intervalChecks();
    updateError();
    intervalChecks();
    calculatePID();
    intervalChecks();
    adjustMotors();
    intervalChecks();
    stop = false;
    reachedSilo = false;
    delay(5);
}