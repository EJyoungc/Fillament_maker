#include <PID_v1.h>


byte NTCPin = A0;
#define SERIESRESISTOR 100000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define BCOEFFICIENT 3950
#define Controller A1
#define Relay 8

double Input;
double Output;
double Kp = 40, Ki = 0, Kd = 0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float sample(byte z) {
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
  return map(knob, 0, 1023, 0, 200);
}

void run() {

  if (getSetpoint() <= 10) {
    // turn off
    //delay(100);
    digitalWrite(Relay, LOW);

  } else {
    // turn on
    if (getSetpoint() <= getTemp()) {
      // turn off
      // delay(100);
      digitalWrite(Relay, LOW);

    } else {
      // turn on
      //delay(100);
      digitalWrite(Relay, HIGH);
    }
  }
}



void setup() {
  Serial.begin(9600);
  pinMode(Relay, OUTPUT);
  myPID.setOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
}
void loop() {

  Input = getTemp();
  Serial.println("temp:" + String(getTemp()) + " Setpoint:" + String(getSetpoint()));
  // delay(100);
  myPID.Compute();
   
}