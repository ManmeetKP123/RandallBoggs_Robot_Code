// #include <Wire.h>
// #include <Adafruit_SSD1306.h>
 
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET  -1 // This display does not have a reset pin accessible
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
// #define REFLECTANCE PA0
 
// void setup() {  
//   pinMode(REFLECTANCE, INPUT);
 
//   //display setup
//   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
//   display.display();
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(SSD1306_WHITE);
//   display.setCursor(0,0);
//   display.println("Hello world!");
//   display.display();
//   delay(2000);
// }
 
// void loop() {
//     display.clearDisplay();
//     display.setCursor(0,0);
//     display.println("Reflectance: ");
//     display.println(analogRead(REFLECTANCE));
//     display.display();
// };
