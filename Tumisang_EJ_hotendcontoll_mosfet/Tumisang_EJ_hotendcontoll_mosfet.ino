#include <PID_v1.h>
byte NTCPin = A0;
#define SERIESRESISTOR 100000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define BCOEFFICIENT 3950
#define Controller A1
#define Hotend 9
byte Setpoint = 0;
double Input;
double Output;
double Kp = 2.0, Ki = 0, Kd = 0;

// Define pin numbers for buttons
const int buttonPinIncrease = 10;
const int buttonPinDecrease = 12;
const int buttonPinSave = 11;

// Variables to store button states
int buttonStateIncrease = 0;
int buttonStateDecrease = 0;
int buttonStateSave = 0;

// Variable to store setpoint value
// byte setpoint = 0;

// Variable to store saved setpoint
double savedSetpoint = 0;

// Debounce settings
unsigned long lastDebounceTimeIncrease = 0;
unsigned long lastDebounceTimeDecrease = 0;
unsigned long lastDebounceTimeSave = 0;
unsigned long debounceDelay = 50;

PID myPID(&Input, &Output, &savedSetpoint, Kp, Ki, Kd, DIRECT);
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

double reverse() {
  return map(savedSetpoint, 0, 255, 255, 0);
}
double finalOutput() {
  return map(Output, 0, 255, 255, 0);
}

void buttonSetting() {
  int readingIncrease = digitalRead(buttonPinIncrease);
  int readingDecrease = digitalRead(buttonPinDecrease);
  int readingSave = digitalRead(buttonPinSave);

  // Increase button logic
  if (readingIncrease == LOW && (millis() - lastDebounceTimeIncrease) > debounceDelay) {
    Setpoint++;
    lastDebounceTimeIncrease = millis();
  }

  // Decrease button logic
  if (readingDecrease == LOW && (millis() - lastDebounceTimeDecrease) > debounceDelay) {
    Setpoint--;
    lastDebounceTimeDecrease = millis();
  }

  // Save button logic
  if (readingSave == LOW && (millis() - lastDebounceTimeSave) > debounceDelay) {
    savedSetpoint = Setpoint;

    lastDebounceTimeSave = millis();
  }

  // Print the current setpoint value
  // Serial.print("Current setpoint: ");
  // Serial.println(setpoint);

  // Small delay to avoid overwhelming the serial output
  delay(200);
}

void run() {

  // Setpoint = getSetpoint();
  // Input = getTemp();

  myPID.Compute();
  analogWrite(Hotend, finalOutput());
  // analogWrite(Hotend, Output);
}



void setup() {
  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  pinMode(buttonPinIncrease, INPUT_PULLUP);
  pinMode(buttonPinDecrease, INPUT_PULLUP);
  pinMode(buttonPinSave, INPUT_PULLUP);
}
void loop() {
  buttonSetting();
  run();

  // Serial.println("temp:" + String(getTemp()) + " Setpoint:" + String(savedSetpoint) + " output:" + String(finalOutput())+ " setting:"+ String(Setpoint));
  Serial.println(  String(getTemp()) + " " + String(savedSetpoint) + " " + String(finalOutput())+ " "+ String(Setpoint));
  // delay(100);
}