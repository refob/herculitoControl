#include <string.h>
#include "config.h"
#include "kinematics.h"
#include "StepperControl.h"
#include "servo_gripper.h"
#include "display.h"
#include "herculito.h"

extern Display display;
extern StepperControl StepperA;
extern StepperControl StepperB;
extern StepperControl StepperC;
#ifdef STEPPER_D
extern StepperControl StepperD;
#endif
extern Servo_Gripper servo_gripper;
extern Servo_Gripper servo_gripper2;

void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WIDTH];
  display.png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  display.tft.pushImage(display.xpos, display.ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}

Display::Display(int rotation) {
  tft = TFT_eSPI();
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  gripro = gripoo = griprn = gripon = 0.0;
  an = bn = cn = dn = aov = bov = cov = dov = 0.0;
  lifebcnt = 0;
  gui_active = false;
};

void Display::life_beat() {
  tft.setTextColor(TFT_LIMEGREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextDatum(BL_DATUM);
  tft.setTextPadding(0);

  switch (lifebcnt) {
    case 0:
      tft.drawString("| ", 76, 128);
      break;
    case 1:
      tft.drawString("/ ", 76, 128);
      break;
    case 2:
      tft.drawString("--", 76, 128);
      break;
    case 3:
      tft.drawString("\\ ", 76, 128);
      break;
    case 4:
      tft.drawString("| ", 76, 128);
      break;
    case 5:
      tft.drawString("/ ", 76, 128);
      break;
    case 6:
      tft.drawString("--", 76, 128);
      break;
    case 7:
      tft.drawString("\\ ", 76, 128);
      break;
  }

  lifebcnt = lifebcnt + 1;
  if (lifebcnt > 7)
    lifebcnt = 0;
}

void Display::output_text(const char *msg) {
  tft.setTextPadding(0);
  tft.setTextSize(3);
  tft.setTextFont(4);
  if (tft.textWidth(msg) > tft.width())
    tft.setTextSize(2);
  if (tft.textWidth(msg) > tft.width())
    tft.setTextSize(1);
  if (tft.textWidth(msg) > tft.width()) {
    tft.setTextFont(2);
  }
  if (tft.textWidth(msg) > tft.width()) {
    tft.setCursor(0, 0);
    tft.println(msg);
  } else {
    tft.setTextDatum(MC_DATUM);
    tft.drawString(msg, tft.width() / 2, tft.height() / 2);
  }
}

void Display::message(const char *msg) {
  gui_active = false;
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  output_text(msg);
}

void Display::warning(const char *msg) {
  gui_active = false;
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_MOCCASIN, TFT_BLACK);
  output_text(msg);
}

void Display::error(const char *msg) {
  gui_active = false;
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  output_text(msg);
}

void Display::herculito() {
  gui_active = false;
  int16_t rc = display.png.openFLASH((uint8_t *)Herculito, sizeof(Herculito), pngDraw);
  display.xpos = display.ypos = 0;
  if (rc == PNG_SUCCESS) {
    //Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
    display.tft.startWrite();
    rc = display.png.decode(NULL, 0);
    display.tft.endWrite();
    display.png.close();
  } else
    Serial.println("Failed to open png file");
}

void Display::status() {
  int x1, y1, x2, inc;
  if ((gui_active)||(!StepperA.power_on)) return;
  gui_active = true;
  x1 = 16;
  x2 = 130;
  y1 = 22;
  inc = 26;
  // Serial.println("status");
  tft.fillScreen(TFT_BLACK);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_GRAY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextFont(4);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("X:", x1, y1);
  tft.drawString("Y:", x1, y1 + inc);
  tft.drawString("Z", x1, y1 + 2 * inc);
  tft.drawString("U:", x1, y1 + 3 * inc);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextDatum(BL_DATUM);

  tft.drawString("G:", 0, 128);
  tft.drawString("R:", 104, 128);

  aov = an + 1;  // force update
  bov = bn + 1;
  cov = cn + 1;
  dov = dn + 1;
  gripoo = gripon + 1.0;
  gripro = griprn + 1.0;
  update_status();
}

void Display::update_status() {
  int x2, y1, inc;
  if ((!gui_active)||(!StepperA.power_on)) return;
  x2 = 130;
  y1 = 22;
  inc = 26;
  //Serial.println("update");
  // Get current values
  display.an = StepperA.currentPosition();
  display.bn = StepperB.currentPosition();
  display.cn = StepperC.currentPosition();
  display.dn = StepperD.currentPosition();
  display.gripon = servo_gripper.get_angle()/1.8;
  display.griprn = servo_gripper2.get_angle();

  if ((an != aov) || (bn != bov) || (cn != cov) || (dn != dov)) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextFont(4);
    tft.setTextPadding(tft.textWidth("-9999.9"));
    tft.setTextDatum(MR_DATUM);
    if ((an != aov) || (bn != bov)) {
      POINT pt = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
      tft.drawFloat(pt.x, 1, x2, y1);
      tft.drawFloat(pt.y, 1, x2, y1 + inc);
      aov = an;
      aov = bn;
    }
    if (dn != dov) {
      tft.drawFloat(StepperD.currentDistance(), 1, x2, y1 + 2 * inc);
      dov = dn;
    }
    if (cn != cov) {
      tft.drawFloat(StepperC.currentAngle(360), 1, x2, y1 + 3 * inc);
      cov = cn;
    }
  }

  if ((gripon != gripoo) || (griprn != gripro)) {
    int tlen1, tlen2;
    tft.setTextColor(TFT_LIMEGREEN, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextFont(2);
    tlen1 = tft.textWidth("X: ");
    tlen2 = tft.textWidth("-999.9");
    tft.setTextPadding(tlen2);
    tft.setTextDatum(BR_DATUM);
    if (gripon != gripoo) {
      tft.drawFloat(gripon, 1, tlen1 + tlen2, 128);
      gripoo = gripon;
    }
    if (griprn != gripro) {
      tft.drawFloat(griprn, 1, 160, 128);
      gripro = griprn;
    }
  }
}
