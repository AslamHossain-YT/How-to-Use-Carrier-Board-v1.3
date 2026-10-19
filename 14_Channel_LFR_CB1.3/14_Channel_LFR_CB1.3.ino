#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);  // Fast I2C / TWI
#include <EEPROM.h>
//motorpins
#define right_motor_forward 10  //IN1(L298) || TB6612 -- AIN2
#define right_motor_backward 11  //IN2(L298) || TB6612 -- AIN1
#define left_motor_forward 3    //IN3(L298)  || TB6612 --- BIN1
#define left_motor_backward 9   //IN4(L298)  || TB6612 --- BIN2
//speed control pins (PWM)
#define right_motor_speed 6  //EnA (L298) //TB6612 --- PWMA
#define left_motor_speed 5   //EnB (L298) //TB6612 --- PWMB
//button pins
#define BUTTON_UP_PIN A2      // pin for UP button
#define BUTTON_SELECT_PIN A0  // pin for SELECT button
#define BUTTON_DOWN_PIN A1    // pin for DOWN button
//MUX PIN
#define S0 4
#define S1 7
#define S2 8
#define S3 12
#define SIG_PIN A7
//Sensor Variables
#define sensorNumber 14
int sensorADC[sensorNumber];
int sensorDigital[sensorNumber];
int bitWeight[sensorNumber] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 };
int WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140 };
int sumOnSensor, sensorWight,bitSensor;
int Max_ADC[sensorNumber],Min_ADC[sensorNumber],Reference_ADC[sensorNumber];
//PID Variables
float line_position,error,center_position = 75,derivative, previous_error;
int base_speed = 150,kp = 5,kd = 100;
//turns variable
String direction = "straight";  //default value straight
//timers
//inverse parameter
bool inverseON = 0;  //0 = normal line, 1 = inverse line
//led
#define led 13

const unsigned char upir_logo[] PROGMEM = {  // this is another way how to define images, using binary notation
  B00010101, B11010111,
  B00010101, B01000101,
  B00010101, B10010110,
  B00011001, B00010101
};

// 'scrollbar_background', 8x64px
const unsigned char bitmap_scrollbar_background[] PROGMEM = {
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00
};
// 'item_sel_outline', 128x21px
const unsigned char bitmap_item_sel_outline[] PROGMEM = {
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0,
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0
};
// ------------------ end generated bitmaps from image2cpp ---------------------------------
const int NUM_ITEMS = 5;         // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH = 20;  // maximum characters for the item name

char menu_items[NUM_ITEMS][MAX_ITEM_LENGTH] = {
  // array with item names
  { "Line Follow" },
  { "Calibration" },
  { "Analog Value" },
  { "Digital Value" },
  { "Motor Test" },
};
// note - when changing the order of items above, make sure the other arrays referencing bitmaps
// also have the same order, for example array "bitmap_icons" for icons, and other arrays for screenshots and QR codes
int button_up_clicked = 0;      // only perform action when button is clicked, and wait until another press
int button_select_clicked = 0;  // same as above
int button_down_clicked = 0;    // same as above
int item_selected = 0;          // which item in the menu is selected
int item_sel_previous;          // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next;              // next item - used in the menu screen to draw next item after the selected one
int current_screen = 0;         // 0 = menu, 1 = screenshot, 2 = qr
//************************************SETUP***************************************
void setup() {
  Serial.begin(9600);
  u8g.setColorIndex(1);  // set the color to white
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  LoadCalibration();  //load calibration value from eeprom whenever the arduino is start or reset
}
//************************************LOOP***************************************
void loop() {
  while (1) {
    read_black_line();
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(5, 15, "RoboTech Innovator");  //Set Your Team Name
      u8g.setFont(u8g_font_profont12);
      u8g.drawStr(10, 15, "");  //Set Your Team Name
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(5, 60);
      u8g.print(sensorDigital[0]);
      u8g.setPrintPos(15, 60);
      u8g.print(sensorDigital[1]);
      u8g.setPrintPos(25, 60);
      u8g.print(sensorDigital[2]);
      u8g.setPrintPos(20, 40);
      u8g.print(sensorDigital[3]);
      u8g.setPrintPos(30, 40);
      u8g.print(sensorDigital[4]);
      u8g.setPrintPos(40, 40);
      u8g.print(sensorDigital[5]);
      u8g.setPrintPos(50, 40);
      u8g.print(sensorDigital[6]);
      u8g.setPrintPos(60, 40);
      u8g.print(sensorDigital[7]);
      u8g.setPrintPos(70, 40);
      u8g.print(sensorDigital[8]);
      u8g.setPrintPos(80, 40);
      u8g.print(sensorDigital[9]);
      u8g.setPrintPos(90, 40);
      u8g.print(sensorDigital[10]);
      u8g.setPrintPos(100, 60);
      u8g.print(sensorDigital[11]);
      u8g.setPrintPos(110, 60);
      u8g.print(sensorDigital[12]);
      u8g.setPrintPos(120, 60);
      u8g.print(sensorDigital[13]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) menu();}}

void PID_Controller(int base_speed, int p, int d) {
  u8g.setFont(u8g_font_7x14B);
  u8g.firstPage();
  do {
    u8g.drawStr(30, 35, "Line Follow");
    u8g.drawStr(40, 50, "Activated");
  } while (u8g.nextPage());
  delay(500);
  while (1) {
    read_black_line();
    if (sumOnSensor > 0) line_position = sensorWight / sumOnSensor;
    error = center_position - line_position;
    derivative = error - previous_error;
    int right_motor_correction = base_speed + (error * p + derivative * d);
    int left_motor_correction = base_speed - (error * p + derivative * d);
    previous_error = error;
    //Drive Motor According to PID Value
    motor(left_motor_correction, right_motor_correction);}}
void calibrateSensor() {
  u8g.setFont(u8g_font_7x14B);  // Set the font
  u8g.firstPage();
  do {
    u8g.drawStr(5, 35, "Calibrating...");
  } while (u8g.nextPage());
  // Initialize calibration limits
  for (int i = 0; i < sensorNumber; i++) {
    Max_ADC[i] = 0;     // Set the maximum ADC value for each sensor to 0 (lowest possible)
    Min_ADC[i] = 1024;  // Set the minimum ADC value for each sensor to 1024 (highest possible, since ADC range is 0–1023)
  }
  // Define rotation directions for calibration sweeps
  int rotationDirections[][2] = {
    { -1, 1 },  // Rotate robot slightly to the left
    { 1, -1 },  // Rotate robot slightly to the right
    { 1, -1 },  // Repeat rotation to the right for better calibration coverage
    { -1, 1 }   // Rotate left again to cover the full sensor range
  };
  // Perform calibration sweeps (4 passes)
  for (int step = 0; step < 4; step++) {
    // Rotate the robot slowly in the defined direction for this step
    motor(base_speed * 0.4 * rotationDirections[step][0],
          base_speed * 0.4 * rotationDirections[step][1]);
    // Sweep through several readings while rotating
    for (int sweep = 0; sweep < 500; sweep++) {
      // Read each sensor one by one
      for (int i = 0; i < sensorNumber; i++) {
        selectChannel(i);                    // Select which sensor channel to read (if using multiplexer)
        sensorADC[i] = analogRead(SIG_PIN);  // Read the analog value from the sensor
        // Update the maximum and minimum ADC values recorded for this sensor
        Max_ADC[i] = max(Max_ADC[i], sensorADC[i]);
        Min_ADC[i] = min(Min_ADC[i], sensorADC[i]);
      }
    }
    motor(0, 0);  // Stop the motors after each rotation step
    delay(200);   // Small pause before changing rotation direction
  }
  // Store calibrated Reference, Max, and Min values in EEPROM
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2.0;  // Calculate mid-point (reference) value
    EEPROM.write(i, Reference_ADC[i] / 4);           // Store reference value (divided by 4 because EEPROM stores 0–255)
    delay(5);                                        // Small delay for EEPROM write stability
    EEPROM.write(i + sensorNumber, Max_ADC[i] / 4);  // Store maximum calibrated value
    delay(5);
    EEPROM.write(i + (sensorNumber * 2), Min_ADC[i] / 4);  // Store minimum calibrated value
    delay(5);
  }
  show_calibration();
}
// Function to load calibration data from EEPROM
void LoadCalibration() {
  // Print header labels for serial monitor (sensor indices)
  Serial.print("S0: ");
  Serial.print(", S1: ");
  Serial.print(", S2: ");
  Serial.print(", S3: ");
  Serial.print(", S4: ");
  Serial.print(", S5: ");
  Serial.print(", S6: ");
  Serial.print(", S7: ");
  Serial.println();
  // Load the stored calibration data for each sensor
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(i) * 4;                 // Read and restore reference value
    Max_ADC[i] = EEPROM.read(i + sensorNumber) * 4;        // Read and restore maximum value
    Min_ADC[i] = EEPROM.read(i + (sensorNumber * 2)) * 4;  // Read and restore minimum value
    Serial.print(String(Reference_ADC[i]) + ",  ");  // Print loaded reference value
  }
  Serial.println();  // Move to new line after printing all sensor values
}
//function for displaying calibration value
void show_calibration() {
  while (1) {
    // Set the font
    u8g.firstPage();
    do {
      //title print
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(15, 10, "MIN");
      u8g.drawStr(55, 10, "MID");
      u8g.drawStr(95, 10, "MAX");
      //value print
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(15, 30);
      u8g.print(Min_ADC[0]);
      u8g.setPrintPos(15, 45);
      u8g.print(Min_ADC[7]);
      u8g.setPrintPos(15, 60);
      u8g.print(Min_ADC[13]);
      //
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(55, 30);
      u8g.print(Reference_ADC[0]);
      u8g.setPrintPos(55, 45);
      u8g.print(Reference_ADC[7]);
      u8g.setPrintPos(55, 60);
      u8g.print(Reference_ADC[13]);
      //
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(95, 30);
      u8g.print(Max_ADC[0]);
      u8g.setPrintPos(95, 45);
      u8g.print(Max_ADC[7]);
      u8g.setPrintPos(95, 60);
      u8g.print(Max_ADC[13]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) return menu();
  }
}
void menu() {
  while (1) {
    if (current_screen == 0) {  // MENU SCREEN
      // up and down buttons only work for the menu screen
      if ((digitalRead(BUTTON_UP_PIN) == LOW) && (button_up_clicked == 0)) {  // up button clicked - jump to previous menu item
        item_selected = item_selected - 1;                                    // select previous item
        button_up_clicked = 1;                                                // set button to clicked to only perform the action once
        if (item_selected < 0) {                                              // if first item was selected, jump to last item
          item_selected = NUM_ITEMS - 1;
        }
      } else if ((digitalRead(BUTTON_DOWN_PIN) == LOW) && (button_down_clicked == 0)) {  // down button clicked - jump to next menu item
        item_selected = item_selected + 1;                                               // select next item
        button_down_clicked = 1;                                                         // set button to clicked to only perform the action once
        if (item_selected >= NUM_ITEMS) {                                                // last item was selected, jump to first menu item
          item_selected = 0;
        }
      }

      if ((digitalRead(BUTTON_UP_PIN) == HIGH) && (button_up_clicked == 1)) {  // unclick
        button_up_clicked = 0;
      }
      if ((digitalRead(BUTTON_DOWN_PIN) == HIGH) && (button_down_clicked == 1)) {  // unclick
        button_down_clicked = 0;
      }
    }


    if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_select_clicked == 0)) {  // select button clicked, jump between screens
      button_select_clicked = 1;                                                    // set button to clicked to only perform the action once
      if (current_screen == 0) { current_screen = 1; }                              // menu items screen --> screenshots screen
      else if (current_screen == 1) {
        current_screen = 2;
      }                             // screenshots screen --> qr codes screen
      else { current_screen = 0; }  // qr codes screen --> menu items screen
    }
    if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_select_clicked == 1)) {  // unclick
      button_select_clicked = 0;
    }

    // set correct values for the previous and next items
    item_sel_previous = item_selected - 1;
    if (item_sel_previous < 0) { item_sel_previous = NUM_ITEMS - 1; }  // previous item would be below first = make it the last
    item_sel_next = item_selected + 1;
    if (item_sel_next >= NUM_ITEMS) { item_sel_next = 0; }  // next item would be after last = make it the first

    //execute specific task when select button is clicked
    if (item_selected == 0 && button_select_clicked == 1) {
      current_screen = 0;
      PID_Controller(base_speed, kp, kd);
    }
    if (item_selected == 1 && button_select_clicked == 1) {
      current_screen = 0;
      calibrateSensor();
    }
    if (item_selected == 2 && button_select_clicked == 1) {
      current_screen = 0;
      analog_value();
    }
    if (item_selected == 3 && button_select_clicked == 1) {
      current_screen = 0;
      digital_value();
    }
    if (item_selected == 4 && button_select_clicked == 1) {
      current_screen = 0;
      motor_test();
    }



    u8g.firstPage();  // required for page drawing mode for u8g library
    do {

      if (current_screen == 0) {  // MENU SCREEN

        // selected item background
        u8g.drawBitmapP(0, 22, 128 / 8, 21, bitmap_item_sel_outline);

        // draw previous item as icon + label
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15, menu_items[item_sel_previous]);
        //u8g.drawBitmapP( 4, 2, 16/8, 16, bitmap_icons[item_sel_previous]);

        // draw selected item as icon + label in bold font
        u8g.setFont(u8g_font_7x14B);
        u8g.drawStr(25, 15 + 20 + 2, menu_items[item_selected]);
        //u8g.drawBitmapP( 4, 24, 16/8, 16, bitmap_icons[item_selected]);

        // draw next item as icon + label
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15 + 20 + 20 + 2 + 2, menu_items[item_sel_next]);
        //u8g.drawBitmapP( 4, 46, 16/8, 16, bitmap_icons[item_sel_next]);

        // draw scrollbar background
        u8g.drawBitmapP(128 - 8, 0, 8 / 8, 64, bitmap_scrollbar_background);

        // draw scrollbar handle
        u8g.drawBox(125, 64 / NUM_ITEMS * item_selected, 3, 64 / NUM_ITEMS);

        // draw upir logo
        u8g.drawBitmapP(128 - 16 - 4, 64 - 4, 16 / 8, 4, upir_logo);
      }
    } while (u8g.nextPage());  // required for page drawing mode with u8g library
  }
}
uint8_t button(uint8_t buttonNumber) {
  uint16_t timer = 0;
  if (!digitalRead(buttonNumber)) {delay(50);
    while (!digitalRead(buttonNumber)) {delay(50);timer += 40;if (timer >= 400) digitalWrite(led, 1);
    }digitalWrite(led, 0);if (timer > 0) return 1;}return 0;}
void motor(int LPWM, int RPWM) {
  if (LPWM > 0) {digitalWrite(left_motor_forward, HIGH);digitalWrite(left_motor_backward, LOW);} else {
    digitalWrite(left_motor_forward, LOW);digitalWrite(left_motor_backward, HIGH);}
  if (RPWM > 0) {digitalWrite(right_motor_forward, HIGH);digitalWrite(right_motor_backward, LOW);} else {
    digitalWrite(right_motor_forward, LOW);digitalWrite(right_motor_backward, HIGH);}
  LPWM = constrain(LPWM, -255, 255);RPWM = constrain(RPWM, -255, 255);
  analogWrite(left_motor_speed, abs(LPWM));analogWrite(right_motor_speed, abs(RPWM));}
//motor test function
void motor_test() {
  while (1) {
    u8g.setFont(u8g_font_7x14B);  // Set the font
    u8g.firstPage();
    do {
      u8g.drawStr(5, 35, "Motor Testing...");
    } while (u8g.nextPage());
    motor(250, 250);
    digitalWrite(led, HIGH);
    delay(3000);
    motor(0, 0);
    digitalWrite(led, LOW);
    return;}}
void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    if (sensorADC[i] > Reference_ADC[i]) {
      if (inverseON == 0) {sensorDigital[i] = 1;} else {sensorDigital[i] = 0;}}
      else {if (inverseON == 0) {sensorDigital[i] = 0;}
      else {sensorDigital[i] = 1;}}
    sumOnSensor += sensorDigital[i];
    sensorWight += sensorDigital[i] * WeightValue[i];
    bitSensor += sensorDigital[i] * bitWeight[(sensorNumber - 1) - i];}}
void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));digitalWrite(S3, bitRead(channel, 3));}

void sensor_value_test() {
  while (1) {
    read_black_line();
    for (int i = 0; i < sensorNumber; i++) {
      Serial.print(String(sensorADC[i]) + "  ");
    }Serial.println();}}

void Bit_Sensor_Show() {
  while (1) {
    read_black_line();  //take sensor reading
    Serial.print("Bit Sensor in Decimal: " + String(bitSensor));  //Decimal Value Show
    Serial.print(" | Bit Sensor in Binary: ");
    for (int i = (sensorNumber - 1); i >= 0; i--) {  //display bitsensor data in binary
      Serial.print(String(bitRead(bitSensor, i)));}Serial.println();}}

void sensor_value_check() {
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    //map analog value
    Serial.print(String(sensorADC[i]) + "  ");}
  for (int i = 0; i < sensorNumber; i++) {
    //analog to digital
    if (sensorADC[i] > Reference_ADC[i]) {sensorDigital[i] = 1;} else {sensorDigital[i] = 0;}
    Serial.print("  " + String(sensorDigital[i]));}Serial.println();}
//function for displaying analog value
void analog_value() {
  while (1) {
    for (int channel = 0; channel < sensorNumber; channel++) {
      selectChannel(channel);
      sensorADC[channel] = analogRead(SIG_PIN);
    }
    u8g.setFont(u8g_font_profont12);  // Set the font
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(5, 15);
      u8g.print(sensorADC[0]);
      u8g.setPrintPos(35, 15);
      u8g.print(sensorADC[1]);
      u8g.setPrintPos(65, 15);
      u8g.print(sensorADC[2]);
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(10, 30);
      u8g.print(sensorADC[3]);
      u8g.setPrintPos(40, 30);
      u8g.print(sensorADC[4]);
      u8g.setPrintPos(70, 30);
      u8g.print(sensorADC[5]);
      u8g.setPrintPos(100, 30);
      u8g.print(sensorADC[6]);
      u8g.setPrintPos(10, 45);
      u8g.print(sensorADC[7]);
      u8g.setPrintPos(40, 45);
      u8g.print(sensorADC[8]);
      u8g.setPrintPos(70, 45);
      u8g.print(sensorADC[9]);
      u8g.setPrintPos(100, 45);
      u8g.print(sensorADC[10]);
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(40, 60);
      u8g.print(sensorADC[11]);
      u8g.setPrintPos(70, 60);
      u8g.print(sensorADC[12]);
      u8g.setPrintPos(100, 60);
      u8g.print(sensorADC[13]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) return;  }}
void digital_value() {
  while (1) {
    read_black_line();
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_profont12);
      u8g.drawStr(30, 15, "");  //Set Your Team Name
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(5, 60);
      u8g.print(sensorDigital[0]);
      u8g.setPrintPos(15, 60);
      u8g.print(sensorDigital[1]);
      u8g.setPrintPos(25, 60);
      u8g.print(sensorDigital[2]);
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(20, 40);
      u8g.print(sensorDigital[3]);
      u8g.setPrintPos(30, 40);
      u8g.print(sensorDigital[4]);
      u8g.setPrintPos(40, 40);
      u8g.print(sensorDigital[5]);
      u8g.setPrintPos(50, 40);
      u8g.print(sensorDigital[6]);
      u8g.setPrintPos(60, 40);
      u8g.print(sensorDigital[7]);
      u8g.setPrintPos(70, 40);
      u8g.print(sensorDigital[8]);
      u8g.setPrintPos(80, 40);
      u8g.print(sensorDigital[9]);
      u8g.setPrintPos(90, 40);
      u8g.print(sensorDigital[10]);
      u8g.setPrintPos(100, 60);
      u8g.print(sensorDigital[11]);
      u8g.setPrintPos(110, 60);
      u8g.print(sensorDigital[12]);
      u8g.setPrintPos(120, 60);
      u8g.print(sensorDigital[13]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) return;}}
    
