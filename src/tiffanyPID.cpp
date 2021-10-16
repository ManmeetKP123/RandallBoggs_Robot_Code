// #include <Wire.h>

// #include <Adafruit_SSD1306.h>

// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET 	-1 // This display does not have a reset pin accessible
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// #define SENSOR0 PA4 //most left
// #define SENSOR1 PA5 //left
// #define SENSOR2 PA6 //middle
// #define SENSOR3 PA7 //right
// #define SENSOR4 PB0 //most right

// #define ARM_PIN PB_9
// #define POPSTICK_PIN PA_8
// #define RAMP_PIN PA_9
// #define PISTON_PIN PB_8
// #define IR_SENSOR PB12

// #define MAX_PWM 2500
// #define MAX_ANGLE 180
// #define MIN_PWM 480
// #define MIN_ANGLE 0

// #define IR_THRESHOLD 650

// #define KP_POT PB1

// #define MOTOR_LEFT_1 PA_0
// #define MOTOR_LEFT_2 PA_1
// #define MOTOR_RIGHT_1 PA_2
// #define MOTOR_RIGHT_2 PA_3

// #define MOTORFREQ 100
// #define MAX_SPEED 4095
// #define MIN_SPEED_FORWARD 1000
// #define MIN_SPEED_REVERSE 800
// #define SPEED 1300
// #define TURN_LEFT_MAX 3
// #define TURN_RIGHT_MAX 3
// #define TURN_SPEED_LEFT 2000
// #define TURN_SPEED_RIGHT 2000

// #define LEFT_ADJUSTMENT 0
// #define RIGHT_ADJUSTMENT 380

// unsigned int reflectance_threshold = 500;

// /*https://www.instructables.com/Line-Follower-Robot-PID-Control-Android-Setup/*/
// std::array<int, 5> sensorValues {0, 0, 0, 0, 0};

// std::array<int, 5> mostLeft {0, 0, 0, 0, 1};
// std::array<int, 5> moreLeft {0, 0, 0, 1, 1};
// std::array<int, 5> left {0, 0, 0, 1, 0};
// std::array<int, 5> bitLeft {0, 0, 1, 1, 0};
// std::array<int, 5> center1 {0, 0, 1, 0, 0};
// std::array<int, 5> center2 {0, 1, 1, 1, 0};
// std::array<int, 5> bitRight {0, 1, 1, 0, 0};
// std::array<int, 5> right {0, 1, 0, 0, 0};
// std::array<int, 5> moreRight {1, 1, 0, 0, 0};
// std::array<int, 5> mostRight {1, 0, 0, 0, 0};

// std::array<int, 5> offPath {0, 0, 0, 0, 0};
// std::array<int, 5> stopTape {1, 1, 1, 1, 1};

// std::array<int, 5> turnRightTape1 {1, 0, 1, 0, 0};
// std::array<int, 5> turnRightTape2 {1, 0, 1, 1, 0};
// std::array<int, 5> turnRightTape3 {1, 0, 0, 1, 0};

// std::array<int, 5> turnLeftTape1 {0, 0, 1, 0, 1};
// std::array<int, 5> turnLeftTape2 {0, 1, 1, 0, 1};
// std::array<int, 5> turnLeftTape3 {0, 1, 0, 0, 1};

// std::array<int, 5> siloStart {1,0,1,0,1};

// volatile int error = 0;
// volatile int P = 0;
// volatile int I = 0;
// volatile int D = 0;
// volatile int PIDvalue = 0;
// volatile int previousError = 0;
// volatile double Kp = 480;
// volatile double Ki = 0;//0.0029;
// volatile double Kd = 0; //6800

// volatile bool stop = false;
// bool reachedSilo = false;
// volatile int turn_left_count = 0;
// volatile int turn_right_count = 0;

// void IR_interrupt();

// int closedArmAngle = 70;
// int openArmAngle = 180;
// int midArmAngle = 100;
// int downRampAngle = 180;
// int upRampAngle = 0;
// int openPistonAngle = 0;
// int closedPistonAngle = 180;
// int openPopAngle = 180;
// int closedPopAngle = 90;

// double slope = (MAX_PWM - MIN_PWM) / (MAX_ANGLE - MIN_ANGLE);

// int getPWM(int angle) {
//     int pwm = (int)(slope * angle + MIN_PWM);

//     if (pwm < MIN_PWM) {
//         return MIN_PWM;
//     }
//     else if (pwm > MAX_PWM) {
//         return MAX_PWM;
//     }

//     return pwm;
// }

// void moveServo(PinName pin, int angle) {
//     pwm_start(pin, 50, getPWM(angle), TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
// }

// void releaseServo(PinName pin) {
//     pwm_stop(pin);
// }

// void readPotentiometer() {
//   Kp = analogRead(KP_POT); //max is 1023, so divide by constant to get range that you want
//   display.setCursor(0,56);
//   display.print("Kp: ");
//   display.print(Kp, 5);
//   display.display();
// }

// void goStraight() {
//     pwm_start(MOTOR_LEFT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
// }

// void move() {
//     moveServo(POPSTICK_PIN, closedPopAngle);
//     delay(2000);
//     moveServo(POPSTICK_PIN, openPopAngle);
//     delay(500);
//     moveServo(ARM_PIN, closedArmAngle);
//     delay(1500);
//     moveServo(ARM_PIN, openArmAngle);
//     delay(2000);
//     moveServo(RAMP_PIN, upRampAngle);
//     delay(2000);
//     moveServo(RAMP_PIN, downRampAngle);
//     delay(1000);
// }

// void pickUpCan() {
//     pwm_start(MOTOR_LEFT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     display.setCursor(0,15);
//     display.println("MOVING");
//     display.display();
//     delay(500);
//     move();
//     goStraight();
//     delay(100);
// }

// void IR_interrupt() {
//   detachInterrupt(digitalPinToInterrupt(IR_SENSOR));
//   display.setCursor(0,45);
//   display.println("INTERRUPT");
//   display.display();
//   static unsigned long lastInterruptTime = 0;
//   unsigned long interruptTime = millis();
//   // If interrupts come faster than 5ms, assume it's a bounce and ignore
//   if (interruptTime - lastInterruptTime > 5) {
//     //do nothing
//   } else {
//     pickUpCan();
//   }
//   lastInterruptTime = interruptTime;
//   pickUpCan();
//   attachInterrupt(digitalPinToInterrupt(IR_SENSOR), IR_interrupt, FALLING);
// }

// void setup() {
//   pinMode(SENSOR0, INPUT);
//   pinMode(SENSOR1, INPUT);
//   pinMode(SENSOR2, INPUT);
//   pinMode(SENSOR3, INPUT);
//   pinMode(SENSOR4, INPUT);

//   pinMode(KP_POT, INPUT_PULLUP);
  
//   pinMode(MOTOR_LEFT_1, OUTPUT);
//   pinMode(MOTOR_LEFT_2, OUTPUT);
//   pinMode(MOTOR_RIGHT_1, OUTPUT);
//   pinMode(MOTOR_RIGHT_2, OUTPUT);
//   // pinMode(IR_SENSOR, INPUT_PULLUP);
//   // pinMode(RAMP_PIN, OUTPUT);
//   // pinMode(ARM_PIN, OUTPUT);
//   // pinMode(PISTON_PIN, OUTPUT);
//   // pinMode(POPSTICK_PIN, OUTPUT);

//   /*  START DISPLAY SETUP */
//   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(SSD1306_WHITE);
 
// //   // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
// //   display.display();
// //   delay(2000);

//   // Displays "Hello world!" on the screen
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(SSD1306_WHITE);
//   display.setCursor(0,0);
//   display.println("Hello world!");
//   display.display();
//   delay(1000);
//   display.clearDisplay();
//   /*  END DISPLAY SETUP */

//     // moveServo(ARM_PIN, openArmAngle);
//     // moveServo(RAMP_PIN, downRampAngle);
//     // moveServo(PISTON_PIN, openPistonAngle);
//     // moveServo(POPSTICK_PIN, openPopAngle);
//     // delay(2000);
//     //attachInterrupt(digitalPinToInterrupt(IR_SENSOR), IR_interrupt, FALLING);
//     readPotentiometer();

//     //goStraight();
// }

// void pushCan() {
//     moveServo(PISTON_PIN, closedPistonAngle);
//     delay(1000);
//     moveServo(PISTON_PIN, openPistonAngle);
//     delay(1200);
// }

// void readSensors() {
//   //value of 1 when there is black tape. Somehow black tape reflects more than the ground, so inverse of what is expected
//   sensorValues[0] = (analogRead(SENSOR0) > 720) ? 1 : 0;
//   sensorValues[1] = (analogRead(SENSOR1) > reflectance_threshold) ? 1 : 0;
//   sensorValues[2] = (analogRead(SENSOR2) > reflectance_threshold) ? 1 : 0;
//   sensorValues[3] = (analogRead(SENSOR3) > reflectance_threshold) ? 1 : 0;
//   sensorValues[4] = (analogRead(SENSOR4) > reflectance_threshold) ? 1 : 0;

// //   display.setCursor(0,8);
// //   display.print(sensorValues[0]);
// //   display.print(sensorValues[1]);
// //   display.print(sensorValues[2]);
// //   display.print(sensorValues[3]);
// //   display.print(sensorValues[4]);

// //   display.setCursor(0,16);
// //   display.print("Sensors: ");
// //   display.print(analogRead(SENSOR0));
// //   display.print(" ");
// //   display.print(analogRead(SENSOR1));
// //   display.print(" ");
// //   display.print(analogRead(SENSOR2));
// //   display.setCursor(0,24);
// //   display.print(analogRead(SENSOR3));
// //   display.print(" ");
// //   display.print(analogRead(SENSOR4));
// }

// bool checkSensors(std::array<int, 5> arrToCheck) {
//     return sensorValues == arrToCheck;
// }

// void updateError() {
//   previousError = error;
//   if (checkSensors(mostLeft)) { //black tape on right side, so robot is too left
//     error = 4;
//   } else if (checkSensors(moreLeft)) {
//     error = 3;
//   } else if (checkSensors(left)) {
//     error = 2;
//   } else if (checkSensors(bitLeft)) {
//     error = 1;
//   } else if (checkSensors(center1) || checkSensors(center2)) {
//     error = 0;
//   } else if (checkSensors(bitRight)) {
//     error = -1;
//   } else if (checkSensors(right)) {
//     error = -2;
//   } else if (checkSensors(moreRight)) {
//     error = -3;
//   } else if (checkSensors(mostRight)) {
//     error = -4;
//   } else if (checkSensors(offPath)) {
//     if (previousError > 0) {
//       error = 5;
//     }
//     if (previousError < 0) {
//       error = -5;
//     }
//   } else if (checkSensors(stopTape)) {
//     stop = true;
//     display.setCursor(0,0);
//     display.print("STOPPPPPP");
//     display.display();
//   } else if (checkSensors(siloStart)) {
//        reachedSilo = true;
//   }

//   if (checkSensors(turnRightTape1) || checkSensors(turnRightTape2) || checkSensors(turnRightTape3)) {
//     turn_right_count++;
//   } else {
//     turn_right_count = 0;
//   }

//   if (checkSensors(turnLeftTape1) || checkSensors(turnLeftTape2) || checkSensors(turnLeftTape3)) {
//     turn_left_count++;
//   } else {
//     turn_left_count = 0;
//   }

//   // display.setCursor(0,16);
//   // display.print("Error: ");
//   // display.print(error);
//   // display.display();
// }

// void calculatePID() {
//   P = error;
//   I = I + error;
//   D = error - previousError;
//   PIDvalue = (int)((Kp*P) + (Ki*I) + (Kd*D));

//   // display.setCursor(0,24);
//   // display.print("PID: ");
//   // display.print(PIDvalue);
//   // display.display();
// }

// int keepInSpeedBounds(int speed) {
//   if (speed > MAX_SPEED) {
//     speed = MAX_SPEED;
//   } else if (speed < -MAX_SPEED) {
//     speed = -MAX_SPEED;
//   } else if (speed >= 0 && speed < MIN_SPEED_FORWARD) {
//     speed = MIN_SPEED_FORWARD;
//   } else if (speed < 0 && speed > -MIN_SPEED_REVERSE) {
//     speed = -MIN_SPEED_REVERSE;
//   }

//   return speed;
// }

// void adjustMotors() {
//   int left_speed = SPEED;
//   int right_speed = SPEED;

//   if (PIDvalue < 0) { //black tape is on the left side, so left wheel needs to turn slower and right wheel faster
//     left_speed = left_speed + PIDvalue;
//     right_speed = right_speed - PIDvalue;
//   }
//   else if (PIDvalue > 0) { //black tape is on the right side, so right wheel needs to turn slower
//     right_speed = right_speed - PIDvalue;
//     left_speed = left_speed + PIDvalue;
//   }

//   // if (turn_left_count >= TURN_LEFT_MAX) {
//   //   left_speed = -SPEED;
//   //   right_speed = SPEED + TURN_SPEED_LEFT;
//   //   display.clearDisplay();
//   //   display.setCursor(0,56);
//   //   display.print("TURNING LEFT!!!!!");
//   //   display.display();
//   // } else if (turn_right_count >= TURN_RIGHT_MAX) {
//   //   left_speed = SPEED + TURN_SPEED_RIGHT;
//   //   right_speed = -SPEED;
//   //   display.clearDisplay();
//   //   display.setCursor(0,56);
//   //   display.print("TURNING RIGHT!!!!!");
//   //   display.display();
//   // }

//   //add adjustments
//   left_speed = left_speed >=0 ? left_speed + LEFT_ADJUSTMENT : left_speed - LEFT_ADJUSTMENT;
//   right_speed = right_speed >= 0 ? right_speed + RIGHT_ADJUSTMENT : right_speed - RIGHT_ADJUSTMENT;

//   left_speed = keepInSpeedBounds(left_speed);
//   right_speed = keepInSpeedBounds(right_speed);

//   if (stop || reachedSilo) {
//     left_speed = 0;
//     right_speed = 0;
//   }

// //   display.setCursor(0,32);
// //   display.print("Left speed: ");
// //   display.print(left_speed);
// //   display.display();
  
// //   display.setCursor(0,40);
// //   display.print("right speed: ");
// //   display.print(right_speed);
// //   display.display();

//   if (left_speed >= 0) {
//     pwm_start(MOTOR_LEFT_1, MOTORFREQ, left_speed, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//   } else if (left_speed < 0) {
//     pwm_start(MOTOR_LEFT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_LEFT_2, MOTORFREQ, -left_speed, RESOLUTION_12B_COMPARE_FORMAT);
//   }
  
//   if (right_speed >= 0) {
//     pwm_start(MOTOR_RIGHT_1, MOTORFREQ, right_speed, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//   } else {
//     pwm_start(MOTOR_RIGHT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_2, MOTORFREQ, -right_speed, RESOLUTION_12B_COMPARE_FORMAT);
//   }

//   if (stop) {
//     delay(10000);
//   }
// }

// void displayTurnCounts() {
//   display.setCursor(0,56);
//   display.print("Turns: ");
//   display.print(turn_left_count);
//   display.print(" ");
//   display.print(turn_right_count);
// }

// void loop() {
//     // display.clearDisplay();
//     // display.setCursor(0,0);
//     // //display.println(analogRead(IR_SENSOR));
//     // display.println(digitalRead(IR_SENSOR));
//     //display.clearDisplay();    //display.setCursor(0,32);
//     //display.println(analogRead(IR_SENSOR));
//     //display.display();    // if (analogRead(IR_SENSOR) < IR_THRESHOLD) {
//     //     pickUpCan();
//     // } else {
//     //readPotentiometer();
//     readSensors();
//     updateError();
//     calculatePID();
//     //displayTurnCounts();
//     adjustMotors();
//     stop = false;
//     reachedSilo = false;
//     //delay(100); 
//     //display.display();
//     //}

//     //display.display();
// };

