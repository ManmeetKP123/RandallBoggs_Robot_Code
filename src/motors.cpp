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

// #define KP_POT PB1

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

// void setup() {
//     pinMode(SENSOR0, INPUT);
//     pinMode(SENSOR1, INPUT);
//     pinMode(SENSOR2, INPUT);
//     pinMode(SENSOR3, INPUT);
//     pinMode(SENSOR4, INPUT);

//     pinMode(KP_POT, INPUT_PULLUP);
    
//     pinMode(MOTOR_LEFT_1, OUTPUT);
//     pinMode(MOTOR_LEFT_2, OUTPUT);
//     pinMode(MOTOR_RIGHT_1, OUTPUT);
//     pinMode(MOTOR_RIGHT_2, OUTPUT);

//     /*  START DISPLAY SETUP */
//     display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(SSD1306_WHITE);
    
//     //   // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
//     //   display.display();
//     //   delay(2000);

//     // Displays "Hello world!" on the screen
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(SSD1306_WHITE);
//     display.setCursor(0,0);
//     display.println("Hello world!");
//     display.display();
//     delay(1000);
//     display.clearDisplay();
//     /*  END DISPLAY SETUP */

//     pwm_start(MOTOR_LEFT_1, MOTORFREQ, 1500, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_LEFT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_1, MOTORFREQ, 1200, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_2, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     delay(3000);
//     pwm_start(MOTOR_LEFT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
//     pwm_start(MOTOR_RIGHT_1, MOTORFREQ, 0, RESOLUTION_12B_COMPARE_FORMAT);
// }

// void loop() {

// };