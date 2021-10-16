// #include <Wire.h>
// #include <Adafruit_SSD1306.h>
 
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET  -1 // This display does not have a reset pin accessible
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
// #define ARM_PIN PA_8
// #define RAMP_PIN PA_9
// #define PISTON_PIN PA_10
// #define FLIPPY_ARM PB_9
// #define IR_SENSOR PB1
 
// #define MAX_PWM 2500
// #define MIN_PWM 500
 
// #define IR_THRESHOLD 300
 
// int closedArmAngle = 14;
// int openArmAngle = 170;
// int closedPistonAngle = 180;
// int openPistonAngle = 0;
// int downRampAngle = 150;
// int upRampAngle = 2;
 
// void handle_interrupt();
 
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
 
// void releaseServo(PinName pin) {
//     pwm_stop(pin);
// }

// void move() {
//     // moveServo(FLIPPY_ARM, 160);
//     // delay(1000);
//     // moveServo(FLIPPY_ARM, 20);
//     // delay(1000);
//     // moveServo(FLIPPY_ARM, 160);
//     // delay(1000);
//     // moveServo(ARM_PIN, openArmAngle);
//     // delay(1000);
//     // moveServo(ARM_PIN, closedArmAngle);
//     // delay(1000);
//     // moveServo(ARM_PIN, openArmAngle);
//     // delay(3000);
//     moveServo(RAMP_PIN, 0);
//     delay(1000);
//     moveServo(RAMP_PIN, 180);
//     delay(1000);
//     // moveServo(PISTON_PIN, closedPistonAngle);
//     // delay(1000);
//     // moveServo(PISTON_PIN, openPistonAngle);
//     // delay(1000);
//     // moveServo(PISTON_PIN, closedPistonAngle);
// }
 
// void setup() {
//     /*  START DISPLAY SETUP */
//     display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(SSD1306_WHITE);
    
//     // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
//     display.display();
//     // Displays "Hello world!" on the screen
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(SSD1306_WHITE);
//     display.setCursor(0,0);
//     display.println("Hello world!");
//     display.display();
//     //delay(1000);
//     display.clearDisplay();
//     /*  END DISPLAY SETUP */
 
//     pinMode(IR_SENSOR, INPUT);
//     pinMode(RAMP_PIN, OUTPUT);
//     pinMode(ARM_PIN, OUTPUT);
//     pinMode(FLIPPY_ARM, OUTPUT);
//     //attachInterrupt(digitalPinToInterrupt(IR_SENSOR), handle_interrupt, FALLING);
//     move();
// }
 
// void loop() {
//     // display.clearDisplay();
//     // display.setCursor(0,0);
//     // display.println(analogRead(IR_SENSOR));
//     // display.display();
 
//     // if (analogRead(IR_SENSOR) < IR_THRESHOLD) {
//     //     display.setCursor(0,15);
//     //     display.println("Can time!!!!!");
//     //     display.display();
//     //     move();
//     // }
// }

