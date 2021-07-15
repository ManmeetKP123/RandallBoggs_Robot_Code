#include <Wire.h>
#include <Servo.h>
#include <Wire.h>

#define SERVO_PIN PA0

Servo myServo;
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//#define OBJECT_SENSOR PA0
#define DETECT_THRESHOLD PA1
#define MOTOR_A1 PA_2
#define MOTOR_B1 PA_3
#define MOTOR_A2 PA_6
#define MOTOR_B2 PA_7
#define MOTOR_FREQ 100
#define POT PA6

// int p;
// int ki = 0.32;
// int d = 1;

// unsigned long currentTime, previousTime;
// double elapsedTime;
// int error;
// int lastError;
// int input, output, setPoint;
// int cumError, rateError;

// void handle_interrupt();
// volatile int i=0;
// volatile int set = 10;
// int duty_cycle;

void setup() {  
  //pinMode(OBJECT_SENSOR, INPUT);
  pinMode(DETECT_THRESHOLD, INPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  myServo.attach(SERVO_PIN);
  pwm_start(MOTOR_A1, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_B1, MOTOR_FREQ, 2000, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_A2, MOTOR_FREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_B2, MOTOR_FREQ, 2000, RESOLUTION_12B_COMPARE_FORMAT);
}

// void handle_interrupt() {
//   i++;
// }

// void pwm_start(PinName pin, uint32_t clock_freq, uint32_t value, TimerCompareFormat_t resolution){
//   duty_cycle = value/resolution;
// }

// int computePID(double inp){
//   currentTime = millis();
//   elapsedTime = (double)(currentTime - previousTime);

//   error = setPoint - inp;
//   cumError += error*elapsedTime;
//   rateError = (error-lastError)/elapsedTime;
//   p = analogRead(POT)/100;

//   int out = p*error + ki*cumError + d*rateError;

//   lastError = error;
//   previousTime = currentTime;

//   return out;
// }
 

void loop() {
  delay(300);
  // input = analogRead(OBJECT_SENSOR);
  // display.setCursor(0,0);
  // display.println(p);
  // display.display();
  // display.setCursor(0,15);
  // display.println(input);
  // display.display();
  // display.clearDisplay();
  // delay(300);
  // output = computePID(input);
  // delay(5000);
  //pwm_start(MOTOR_B, MOTOR_FREQ, output, RESOLUTION_12B_COMPARE_FORMAT);
};