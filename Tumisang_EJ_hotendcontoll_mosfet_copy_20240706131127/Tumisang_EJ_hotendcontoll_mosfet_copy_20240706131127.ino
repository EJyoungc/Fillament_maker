#include <LiquidCrystal.h>
byte NTCPin = A0;
#define SERIESRESISTOR 100000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define BCOEFFICIENT 3950
#define Controller A1
#define Hotend 9
#define Fan 8
byte Setpoint = 0;
const long interval = 500;  // interval at which to blink (milliseconds)
unsigned long previousMillis = 0;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 8, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


// Define pin numbers for buttons
const int buttonPinIncrease = 6;
const int buttonPinDecrease = 3;
const int buttonPinSave = 7;

// Variables to store button states
int buttonStateIncrease = 0;
int buttonStateDecrease = 0;
int buttonStateSave = 0;


// Variable to store saved setpoint
double savedSetpoint = 0;
volatile float Error;
// Debounce settings
unsigned long lastDebounceTimeIncrease = 0;
unsigned long lastDebounceTimeDecrease = 0;
unsigned long lastDebounceTimeSave = 0;
unsigned long debounceDelay = 200;
byte tempSet = 0;


float sample(byte z)

{
  byte i;
  float sval = 0;
  for (i = 0; i < 100; i++) {
    sval = sval + analogRead(z);  // sensor on analog pin 'z'
  }
  sval = sval / 100;  // average
  return sval;
}

float getTemp() {
  float ADCvalue;
  float Resistance;
  ADCvalue = sample(NTCPin);
  //convert value to resistance
  Resistance = (1023 / ADCvalue) - 1;
  Resistance = SERIESRESISTOR / Resistance;

  float steinhart;
  steinhart = Resistance / NOMINAL_RESISTANCE;        // (R/Ro)
  steinhart = log(steinhart);                         // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                          // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                        // Invert
  return steinhart -= 273.15;                         // convert to C
}

int getSetpoint() {

  int knob = analogRead(Controller);
  return map(knob, 0, 1023, 290, 0);
}




float error() {
  Error = savedSetpoint - getTemp();
  return Error;
}
void runHotend(float seconds) {
  if (seconds < -2) {
    analogWrite(Hotend, 255);
    digitalWrite(Fan, HIGH);
  } else if (seconds < 2 && seconds > -2) {
    analogWrite(Hotend, 255);
    digitalWrite(Fan, HIGH);

  } else {
    analogWrite(Hotend, 0);
    digitalWrite(Fan, LOW);
    delay(seconds * 10);
  }
}


void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print("MicroMek LTD");
  lcd.setCursor(0, 1);
  lcd.print("Lets Recycle");
  delay(8000);
  pinMode(buttonPinIncrease, INPUT_PULLUP);
  pinMode(buttonPinDecrease, INPUT_PULLUP);
  pinMode(buttonPinSave, INPUT_PULLUP);
  pinMode(Fan, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPinDecrease), reset, FALLING);
}

void reset() {

  savedSetpoint = 0;
  Setpoint = 0;
  Serial.println("reset");
}

void button() {
  int readingIncrease = digitalRead(buttonPinIncrease);
  int readingDecrease = digitalRead(buttonPinDecrease);
  int readingSave = digitalRead(buttonPinSave);
  // Serial.println(readingIncrease);

  // Increase button logic
  if (readingIncrease == LOW && (millis() - lastDebounceTimeIncrease) > debounceDelay) {
    Setpoint++;
    Serial.println("pressed");
    lastDebounceTimeIncrease = millis();
  }

  // Decrease button logic
  // if (readingDecrease == LOW && (millis() - lastDebounceTimeDecrease) > debounceDelay) {
  //   Setpoint--;

  //   lastDebounceTimeDecrease = millis();
  // }

  // Save button logic
  if (readingSave == LOW && (millis() - lastDebounceTimeSave) > debounceDelay) {
    savedSetpoint = Setpoint;
    
    Serial.print("Setpoint saved: ");
    // Serial.println(savedSetpoint);
    lastDebounceTimeSave = millis();
  }
}


void loop() {

  runHotend(error());
  button();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    lcd.begin(16, 2);
    lcd.print("T:" + String(getTemp()) + " S:" + String(savedSetpoint));
    // lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Set:" + String(Setpoint));
  }

  // lcd.clear();
  Serial.println(String(getTemp()) + " " + String(savedSetpoint) + String(tempSet) + " " + String(Error));
}