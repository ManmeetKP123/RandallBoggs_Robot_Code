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

// #define FLIPPY_ARM PB_9
// #define ARM_PIN PA_8
// #define RAMP_PIN PA_9
// #define PISTON_PIN PA_10

// #define IR_SENSOR PB1
// #define MAX_PWM 2500
// #define MIN_PWM 500

// #define MOTOR_LEFT_1 PA_0
// #define MOTOR_LEFT_2 PA_1
// #define MOTOR_RIGHT_1 PA_2
// #define MOTOR_RIGHT_2 PA_3

// #define MOTORFREQ 100
// #define MAX_SPEED 4095
// #define MIN_SPEED_FORWARD 1000
// #define MIN_SPEED_REVERSE 800
// #define MIN_SPEED 
// #define SPEED 1200
// #define TURN_LEFT_MAX 3
// #define TURN_RIGHT_MAX 3
// #define TURN_SPEED_LEFT 2000
// #define TURN_SPEED_RIGHT 2000

// #define LEFT_ADJUSTMENT 180
// #define RIGHT_ADJUSTMENT 0

// #define IR_THRESHOLD 300
 
// int closedArmAngle = 14;
// int openArmAngle = 170;
// int closedPistonAngle = 0;
// int openPistonAngle = 180;
// int downRampAngle = 150;
// int upRampAngle = 2;
// int openFlippyArm = 60;
// int closedFlippyArm = 160;
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

// volatile int error = 0;
// volatile int P = 0;
// volatile int I = 0;
// volatile int D = 0;
// volatile int PIDvalue = 0;
// volatile int previousError = 0;
// volatile double Kp = 490; //490
// volatile double Ki = 0;//0.003;
// volatile double Kd = 5500;//5500

// volatile bool stop = false;
// bool reachedSilo = false;
// volatile bool canDetected = false;
// volatile int turn_left_count = 0;
// volatile int turn_right_count = 0;

// // void readPotentiometer() {
// //   Kd = analogRead(KP_POT)*10; //max is 1023, so divide by constant to get range that you want
// //   display.setCursor(0,56);
// //   display.print("Kd: ");
// //   display.print(Kd, 10);
// //   display.display();
// // }

// int getPWM(int angle) {
//     int pwm = (int)(11.1111 * angle + 500);
 
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

// void move() {
//     moveServo(FLIPPY_ARM, closedFlippyArm);
//     delay(1000);
//     moveServo(FLIPPY_ARM, openFlippyArm);
//     delay(1000);
//     moveServo(ARM_PIN, closedArmAngle);
//     delay(1000);
//     moveServo(ARM_PIN, openArmAngle);
//     delay(3000);
//     moveServo(RAMP_PIN, upRampAngle);
//     delay(1000);
//     moveServo(RAMP_PIN, downRampAngle);
//     delay(1000);
//     // moveServo(PISTON_PIN, closedPistonAngle);
//     // delay(1000);
//     // moveServo(PISTON_PIN, openPistonAngle);
//     // delay(1000);
//     // moveServo(PISTON_PIN, closedPistonAngle);
// }

// void pickUpCan() {
//     pwm_start(MOTOR_LEFT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     delay(100);
//     move();
//     delay(100);
// }

// void setup() {
//   pinMode(SENSOR0, INPUT);
//   pinMode(SENSOR1, INPUT);
//   pinMode(SENSOR2, INPUT);
//   pinMode(SENSOR3, INPUT);
//   pinMode(SENSOR4, INPUT);

//   pinMode(IR_SENSOR, INPUT_PULLUP);
//   pinMode(RAMP_PIN, OUTPUT);
//   pinMode(ARM_PIN, OUTPUT);
//   pinMode(FLIPPY_ARM, OUTPUT);
//   pinMode(PISTON_PIN, OUTPUT);

//   //pinMode(KP_POT, INPUT_PULLUP);
  
//   pinMode(MOTOR_LEFT_1, OUTPUT);
//   pinMode(MOTOR_LEFT_2, OUTPUT);
//   pinMode(MOTOR_RIGHT_1, OUTPUT);
//   pinMode(MOTOR_RIGHT_2, OUTPUT);

//   /*  START DISPLAY SETUP */
//   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(SSD1306_WHITE);
 
//   // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
//   display.display();
//   //delay(2000);

//   //Displays "Hello world!" on the screen
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(SSD1306_WHITE);
//   display.setCursor(0,0);
//   display.println("Hello world!");
//   display.display();
//   delay(1000);
//   display.clearDisplay();
// //   moveServo(ARM_PIN, closedArmAngle);
// //   delay(10);
// //   moveServo(ARM_PIN, openArmAngle);
// //   delay(10);
// //   moveServo(FLIPPY_ARM, openFlippyArm);
// //   delay(10);
// //   moveServo(PISTON_PIN, openPistonAngle);
// //   delay(10);
//   /*  END DISPLAY SETUP */
//   //readPotentiometer();
// moveServo(ARM_PIN, 130);
// }

// void pushCan(){
//     moveServo(PISTON_PIN, closedPistonAngle);
//     delay(1000);
//     moveServo(PISTON_PIN, openPistonAngle);
//     delay(1000);
//     moveServo(PISTON_PIN, closedPistonAngle);
// }

// void readSensors() {
//   //value of 1 when there is black tape. Somehow black tape reflects more than the ground, so inverse of what is expected
//   sensorValues[0] = (analogRead(SENSOR0) > reflectance_threshold) ? 1 : 0;
//   sensorValues[1] = (analogRead(SENSOR1) > reflectance_threshold) ? 1 : 0;
//   sensorValues[2] = (analogRead(SENSOR2) > reflectance_threshold) ? 1 : 0;
//   sensorValues[3] = (analogRead(SENSOR3) > reflectance_threshold) ? 1 : 0;
//   sensorValues[4] = (analogRead(SENSOR4) > 740) ? 1 : 0;

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
//       error = -7;
//     }
//   } else if (checkSensors(stopTape)) {
//     stop = true;
//     // display.setCursor(0,0);
//     // display.print("STOPPPPPP");
//     // display.display();
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

//   left_speed = left_speed >=0 ? left_speed + LEFT_ADJUSTMENT : left_speed - LEFT_ADJUSTMENT;
//   right_speed = right_speed >= 0 ? right_speed + RIGHT_ADJUSTMENT : right_speed - RIGHT_ADJUSTMENT;

//   left_speed = keepInSpeedBounds(left_speed);
//   right_speed = keepInSpeedBounds(right_speed);

//   if (stop) {
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
//     delay(1000);
//   }
// }

// // void displayTurnCounts() {
// //   display.setCursor(0,56);
// //   display.print("Turns: ");
// //   display.print(turn_left_count);
// //   display.print(" ");
// //   display.print(turn_right_count);
// // }

// void checkAndPickUpCan(){
//     if(digitalRead(IR_SENSOR) == 0){
//         pickUpCan();
//     }
// }

// void loop() {
//     //display.setCursor(0,56);
//     // display.println(analogRead(IR_SENSOR));
//     // delay(100);
//     //checkAndPickUpCan();
//     readSensors();
//     //checkAndPickUpCan();
//     updateError();
//     // checkAndPickUpCan();
//     calculatePID();
//     // checkAndPickUpCan();
//     adjustMotors();
//     // display.display();
//     // display.clearDisplay();
//     stop = false;
// };