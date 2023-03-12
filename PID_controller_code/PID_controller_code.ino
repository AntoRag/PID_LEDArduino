#include <Arduino.h>

#define Kd 3
#define Kp 3.5
#define Ki 0.5

#define SAMPLE_TIME 100

#define P_ON 1
#define D_ON 1
#define I_ON 1

// #define REFERENCE 200
#define PIN_READ A0
#define PIN_WRITE 9
#define PIN_REF A8
// 25k ohm buio
// 100 ohm luce
double REFERENCE;
unsigned long lastTime;

double PID_Compute(double read, unsigned long timeChange, double prev_read)
{
  double error = REFERENCE - read;
  double prev_error = REFERENCE - prev_read;
  double e_p = 0;
  double e_d = 0;
  double e_i = 0;
  double output = 0;

  // Compute proportional
  if (P_ON)
  {
    e_p = Kp * error;
    output += e_p;
  }
  // compute incremento finito
  if (D_ON)
  {
    e_d = (error - prev_error) / ((double)timeChange) * Kd * SAMPLE_TIME;
    output += e_d;
  }
  // compute integrative error
  if (I_ON)
  {
    e_i = e_i + (error) * ((double)timeChange) * Ki/SAMPLE_TIME;
    if (abs(error)<6){
      e_i = 0;
    }
    output += e_i;
  }
  output = constrain(output, 0, 255);
  return output;
}

void getREF()
{
  REFERENCE = analogRead(PIN_REF);
  REFERENCE = map(REFERENCE, 0, 1023, 0, 255);
  REFERENCE = constrain(REFERENCE, 0, 255);
}

double getREAD()
{
  double read;
  read = analogRead(PIN_READ);
  read = map(read, 20, 900, 0, 255);
  read = constrain(read, 0, 255);
  return read;
}


void setup()
{
  // put your setup code here, to run once:
  Serial.begin(250000);
}

void loop()
{

  // get current time
  static double prev_read = 0;
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (timeChange >= SAMPLE_TIME)
  {
    double read = getREAD();
    getREF();
    int command = PID_Compute(read, timeChange, prev_read);
    analogWrite(PIN_WRITE, command);
    prev_read = read;
    Serial.print(read);
    Serial.print(",");
    Serial.print(REFERENCE);
    Serial.print(",");
    Serial.println(command);
    // Write from 0 to 255
    analogWrite(PIN_WRITE, command);
  }
}
