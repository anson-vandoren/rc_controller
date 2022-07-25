#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// rotary encoder pins
#define ENC_PIN_1 2
#define ENC_PIN_2 4

// left joystick pins
#define JOY_L_X A2
#define JOY_L_Y A3
#define JOY_R_X A0
#define JOY_R_Y A1

#define OLED_RESET -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct XYPos {
  int x;
  int y;
};

struct XPos {
  int val;
};

XYPos pos1 = {0, 0};
XYPos pos2 = {0, 0};
XPos pos3 = {10};

// rotary encoder position
int enc_pos;
int enc_last_pos;

// joystick positions
int joy_l_x;
int joy_l_y;
int joy_l_trim_x = 0;
int joy_l_trim_y = 0;
int joy_r_x;
int joy_r_y;
int joy_r_trim_x = 0;
int joy_r_trim_y = 0;
#define JOY_ALPHA 0.8 // 0.0 - 1.0, higher = more smoothing

void readEncoderPos();

void displayPositions(XYPos pos1, XYPos pos2, XPos pos3, bool inverted);

void setup()
{
  Serial.begin(9600);

  // set up rotary encoder
  pinMode(ENC_PIN_1, INPUT_PULLUP);
  pinMode(ENC_PIN_2, INPUT_PULLUP);
  enc_last_pos = digitalRead(ENC_PIN_1);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), readEncoderPos, CHANGE);

  // set up joysticks
  pinMode(JOY_L_X, INPUT_PULLUP);
  pinMode(JOY_L_Y, INPUT_PULLUP);
  pinMode(JOY_R_X, INPUT_PULLUP);
  pinMode(JOY_R_Y, INPUT_PULLUP);

  joy_l_x = analogRead(JOY_L_X);
  joy_l_y = analogRead(JOY_L_Y);
  joy_r_x = analogRead(JOY_R_X);
  joy_r_y = analogRead(JOY_R_Y);
  joy_l_trim_x = joy_l_x - 512;
  joy_l_trim_y = joy_l_y - 512;
  joy_r_trim_x = joy_r_x - 512;
  joy_r_trim_y = joy_r_y - 512;
  joy_l_x = 512;
  joy_l_y = 512;
  joy_r_x = 512;
  joy_r_y = 512;

  Serial.println("Starting...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println("SSD1306 allocation failed");
    for (;;); // Don't proceed, loop forever
  }
  Serial.println("SSD1306 allocation successful");
  display.clearDisplay();
  display.display();
  displayPositions(pos1, pos2, pos3, true);
}

void pollJoysticks() {
  int new_l_x = analogRead(JOY_L_X) - joy_l_trim_x;
  int new_l_y = 1023 - analogRead(JOY_L_Y) + joy_l_trim_y; // invert Y
  joy_l_x = joy_l_x * JOY_ALPHA + new_l_x * (1.0 - JOY_ALPHA);
  joy_l_y = joy_l_y * JOY_ALPHA + new_l_y * (1.0 - JOY_ALPHA);
  joy_l_x = constrain(joy_l_x, 0, 1023);
  joy_l_y = constrain(joy_l_y, 0, 1023);
  pos1 = {joy_l_x, joy_l_y};
  int new_r_x = analogRead(JOY_R_X) - joy_r_trim_x;
  int new_r_y = 1023 - analogRead(JOY_R_Y) + joy_r_trim_y; // invert Y
  joy_r_x = joy_r_x * JOY_ALPHA + new_r_x * (1.0 - JOY_ALPHA);
  joy_r_y = joy_r_y * JOY_ALPHA + new_r_y * (1.0 - JOY_ALPHA);
  joy_r_x = constrain(joy_r_x, 0, 1023);
  joy_r_y = constrain(joy_r_y, 0, 1023);
  pos2 = {joy_r_x, joy_r_y};
}

void loop() {
  pollJoysticks();

  displayPositions(pos1, pos2, pos3, true);
}

#define CH_CX_A 28 // x-coordinate of the left-side crosshair center
#define CH_CX_B 81 // x-coordinate of the right-side crosshair center
#define CH_CY 32 // y-coordinate of the crosshairs center (shared)
#define CH_L 49 // length of the crosshairs
#define CH_HM 3 // horizontal margin of the crosshairs
#define CH_VM 7 // vertical margin of the crosshairs
#define CH_RS 2 * CH_HM + 2 * CH_L + 4 // right side of the crosshair space

// scaling values
#define CH_MAX_X 1023 // set to the maximum positive x-value that the controller can report
#define CH_MIN_X 0 // set to the maximum negative x-value that the controller can report
#define CH_MAX_Y 1023 // set to the maximum positive y-value that the controller can report
#define CH_MIN_Y 0 // set to the maximum negative y-value that the controller can report
#define SCALAR_MAX 100 // set to the maximum scalar value that the controller can report


void drawCrosshairs() {
  // left crosshair
  display.drawLine(CH_CX_A, CH_VM, CH_CX_A, CH_VM + CH_L, WHITE); // vertical
  display.drawLine(CH_HM, CH_CY, CH_HM + CH_L, CH_CY, WHITE); // horizontal
  // right crosshair
  display.drawLine(CH_CX_B, CH_VM, CH_CX_B, CH_VM + CH_L, WHITE); // vertical
  display.drawLine(CH_RS - CH_HM, CH_CY, CH_RS - CH_HM - CH_L, CH_CY, WHITE); // horizontal
}

void drawBall(XYPos pos, uint8_t centerX) {
  uint8_t scaledY = map(pos.y, CH_MIN_Y, CH_MAX_Y, -CH_L / 2, CH_L / 2);
  uint8_t scaledX = map(pos.x, CH_MIN_X, CH_MAX_X, -CH_L / 2, CH_L / 2);
  uint8_t drawX = centerX + scaledX;
  uint8_t drawY = CH_CY + scaledY;
  display.fillCircle(drawX, drawY, 5, WHITE);
}

void drawRHBox() {
  // draw the box to hold the scalar value
  display.drawRect(CH_RS + 1, 1, 19, 63, WHITE);
}

void drawScalar(uint8_t val) {
  // scale the value to the box
  uint8_t scaledVal = map(val, 0, SCALAR_MAX, 0, 59);
  // draw the scalar value
  display.fillRect(CH_RS + 3, 62 - scaledVal, 15, scaledVal, WHITE);
}

void displayPositions(XYPos pos1, XYPos pos2, XPos pos3, bool inverted) {
  display.setRotation(inverted ? 2 : 0);
  display.clearDisplay();
  drawCrosshairs();

  drawBall(pos1, CH_CX_A);
  drawBall(pos2, CH_CX_B);

  drawRHBox();
  drawScalar(pos3.val);

  display.display();
}

void readEncoderPos() {
  enc_pos = digitalRead(ENC_PIN_1);
  // only update if the encoder has moved to prevent bouncing
  if (enc_pos == enc_last_pos) {
    return;
  }
  enc_last_pos = enc_pos;
  if (digitalRead(ENC_PIN_2) == enc_pos) {
    pos3.val += 1;
  }
  else {
    pos3.val -= 1;
  }
  if (pos3.val < 0) {
    pos3.val = 0;
  }
  if (pos3.val > SCALAR_MAX) {
    pos3.val = SCALAR_MAX;
  }
  enc_last_pos = enc_pos;
}
