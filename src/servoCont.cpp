// #include <Wire.h>
// #include <Adafruit_SSD1306.h>
 
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET  -1 // This display does not have a reset pin accessible
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
// #define MAX_PWM 2500
// #define MIN_PWM 500

// #define CONT_SERVO PB_8
 
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

// void setup() {  
//   pinMode(CONT_SERVO, OUTPUT);
//   pwm_start(CONT_SERVO, 50, 1480, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
// }

// void loop() {
// };

