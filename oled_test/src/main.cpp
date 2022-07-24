#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// rotary encoder pins
#define ENC_PIN_1 2
#define ENC_PIN_2 4

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

  delay(100);
  Serial.println("Starting...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println("SSD1306 allocation failed");
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display();
  displayPositions(pos1, pos2, pos3, true);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
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
#define CH_MAX_X 100 // set to the maximum positive x-value that the controller can report
#define CH_MAX_Y 100 // set to the maximum positive y-value that the controller can report
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
  display.fillCircle(pos.x + centerX, pos.y + CH_CY, 5, WHITE);
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
