#include <LiquidCrystal.h>
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

#define FLEX_THUMB  A0
#define FLEX_INDEX  A1
#define FLEX_MIDDLE A2
#define FLEX_RING   A3
#define FLEX_PINKY  A4

void setup()
{
  lcd.begin(16, 2);
  lcd.print("    .    .");
  lcd.setCursor(0,1);
  lcd.print("    .    .");
}

void loop()
{
  int angleThumb; int aTMapped;
  int angleIndex; int aIMapped;
  int angleMiddle; int aMMapped;
  int angleRing; int aRMapped;
  int anglePinky; int aPMapped;
  angleThumb = analogRead(FLEX_THUMB);
  aTMapped = map(angleThumb, 130, 390, 0, 90);
  lcd.setCursor(0,0);
  if(aTMapped>9) {
    lcd.print(aTMapped);
  }
  else {
    lcd.print(" ");
    lcd.print(aTMapped);
  }
  angleIndex = analogRead(FLEX_INDEX);
  aIMapped = map(angleIndex, 120, 600, 0, 90);
  lcd.setCursor(5,0);
  if(aIMapped>9) {
    lcd.print(aIMapped);
  }
  else {
    lcd.print(" ");
    lcd.print(aIMapped);
  }
  angleMiddle = analogRead(FLEX_MIDDLE);
  aMMapped = map(angleMiddle, 140, 630, 0, 90);
  lcd.setCursor(10,0);
  if(aMMapped>9) {
    lcd.print(aMMapped);
  }
  else {
    lcd.print(" ");
    lcd.print(aMMapped);
  }
  angleRing = analogRead(FLEX_RING);
  aRMapped = map(angleRing, 110, 500, 0, 90);
  lcd.setCursor(0,1);
  if(aRMapped>9) {
    lcd.print(aRMapped);
  }
  else {
    lcd.print(" ");
    lcd.print(aRMapped);
  }
  anglePinky = analogRead(FLEX_PINKY);
  aPMapped = map(anglePinky, 110, 460, 0, 90);
  lcd.setCursor(5,1);
  if(aPMapped>9) {
    lcd.print(aPMapped);
  }
  else {
    lcd.print(" ");
    lcd.print(aPMapped);
  }
}
