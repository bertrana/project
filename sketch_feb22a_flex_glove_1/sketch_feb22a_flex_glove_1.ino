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
  lcd.setCursor(0,0);
  lcd.print(angleThumb);

  angleIndex = analogRead(FLEX_INDEX);
  lcd.setCursor(5,0);
  lcd.print(angleIndex);

  angleMiddle = analogRead(FLEX_MIDDLE);
  lcd.setCursor(10,0);
  lcd.print(angleMiddle);

  angleRing = analogRead(FLEX_RING);
  lcd.setCursor(0,1);
  lcd.print(angleRing);

  anglePinky = analogRead(FLEX_PINKY);
  lcd.setCursor(5,1);
  lcd.print(anglePinky);
}
