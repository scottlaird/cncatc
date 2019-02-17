/*
  

*/

// include the library code:
#include <LiquidCrystal.h>

// Input defs
const int pinDcAmps=A0;
const int pinSpindleAmps=A1;
const int pinVacAmps=A2;
const int pinCompressorAmps=A3;
const int pinCncSpindleSpeed=A4;

const int pinCncSpindleDirection=2; // M3 on GRBL sets this to 0, M4 sets to 1.
const int pinCncCoolant=3;  // M8 on GRBL sets this to 0, M9 sets this to 1.
const int pinSpindleOnOff=4;
const int pinAirA=5;
const int pinAirB=6;
const int pinAirC=7;



// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
LiquidCrystal lcd(38,39,40,41,42,43);

long totalVolts;
int countVolts;

// Calculate amps given a raw analog reading from an ACS712 current
// measurement IC.  These come in a few different sizes (5, 20, 30
// amps), and need different constants for each, so for now this just
// takes the amperage of the ACS712 as a parameter.
//
// Mostly from http://henrysbench.capnfatz.com/henrys-bench/arduino-current-measurements/the-acs712-current-sensor-with-an-arduino/
//
// Performance note: it looks like the Arduino's FP performance isn't
// as horrible as I'd expected.  Random posts on the Internet suggest
// that double operations take about 9x as long as int operations, but
// long int operations take ~4.5x as long as int operations.  So, this
// would be faster if we did it all with long ints, but it's only a 2x
// perf difference, and it'd take a lot longer to debug and deal with
// overflow cases.
double rawToAmps(int raw, int acs712_amps) {
  int mVperAmp = 185;
  int ACSoffset = 2582;  // Averaged across several devices; should be 2500.
  double voltage;
  double amps;

  switch (acs712_amps) {
  case 5:
    mVperAmp = 185;
    break;
  case 20:
    mVperAmp = 100;
    break;
  case 30:
    mVperAmp = 66;
  default:
    /* No good way to do error handling here... */
    break;
  }
  //Serial.print(" r=");
  //Serial.print(raw);
  voltage = raw / 1024.0 * 5000;
  //Serial.print(" v=");
  //Serial.print(voltage);
  amps = (voltage - ACSoffset) / mVperAmp;
  //Serial.print(" a=");
  //Serial.println(amps);

  //countVolts++;
  //totalVolts+=voltage;
  
  return amps;
}

double decayingAverage(double a, double b, double fraction) {
  return a * fraction + b * (1 - fraction);
}

// Spindle speed notes:
// 48 when CNC board powered totally off.
//
// Values measures with 0.1 weighted average, 100ms sample period.
// GRBL has min speed set to 1000 and max speed set to 25000.  I
// tried setting the minimum GRBL speed to 5000, but that left
// almost no visible margin between 5000 RPM and off.  Reducing the
// minimum in GRBL makes it much clearer.
//
// ~508 when off (S0)
// ~522 when 5000 (S5000)
// ~528 when 6000 (S6000)
// ~544 when 10000
// ~580 when 20000
// ~594 when 25000 (full speed).
//
// So, over 20k RPM, we vary ~73 points, or 3.65 points per 1k RPM.
// If it's linear, then v = (r/1000 - 5) * 3.65 + 525.  That'd put
// 10k around 541, which is a bit low, but not horrible.  20k looks
// okay for that.  Going the other way (which is really what we care
// about), r = round((v-523)/3.7+5)*1000.  That rounds to the
// nearest 1k RPM, which might be best.  Or maybe not.  Maybe just
// for displaying?

const double v5000=521.3;
const double v25000=590.8;
const double v_off=512;

int analogToRpm(double v){
  if (v < v_off)
    return 0; // Off
      
  return ((v-v5000)/((v25000-v5000)/20.0)+5)*1000;
}

void setup() {
  // Set up logging over USB
  Serial.begin(115200);

  // Set up pins
  pinMode(pinSpindleOnOff, OUTPUT);
  pinMode(pinAirA, OUTPUT);
  pinMode(pinAirB, OUTPUT);
  pinMode(pinAirC, OUTPUT);

  //pinMode(pinCncCoolant, INPUT_PULLUP);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.print("System starting.");
}

double dcAmps, spindleAmps, vacAmps, compressorAmps;
double avgSpindleSpeed;

void loop() {
  //int spindleEnabled, spindleSpeed;
  int rpm;

  int cncCoolant, cncSpindleDirection, cncSpindleSpeed;
  double new_dcAmps, new_spindleAmps, new_vacAmps, new_compressorAmps;
  int airState = 0; // (millis()/10000)%2;  // Alternate 0/1 every 10s.

  new_dcAmps = rawToAmps(analogRead(pinDcAmps), 20);
  new_spindleAmps = rawToAmps(analogRead(pinSpindleAmps), 5);
  new_vacAmps = rawToAmps(analogRead(pinVacAmps), 20);
  new_compressorAmps = rawToAmps(analogRead(pinCompressorAmps), 20);

  
  cncSpindleSpeed=analogRead(pinCncSpindleSpeed);
  cncSpindleDirection=digitalRead(pinCncSpindleDirection);
  cncCoolant=digitalRead(pinCncCoolant);
  avgSpindleSpeed=decayingAverage(cncSpindleSpeed, avgSpindleSpeed, 0.2);

  dcAmps = decayingAverage(new_dcAmps, dcAmps, 0.2);
  spindleAmps = decayingAverage(new_spindleAmps, spindleAmps, 0.2);
  vacAmps = decayingAverage(new_vacAmps, vacAmps, 0.2);
  compressorAmps = decayingAverage(new_compressorAmps, compressorAmps, 0.2);
  
  lcd.clear();
  lcd.print("DC: ");
  lcd.print(dcAmps, 3);
  lcd.print("A");

  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
  
  Serial.println(millis() / 1000);
  Serial.print("DC Amps: ");
  Serial.println(dcAmps);
  Serial.print("Spindle Amps: ");
  Serial.println(spindleAmps);
  Serial.print("Vacuum Amps: ");
  Serial.println(vacAmps);
  Serial.print("Compressor Amps: ");
  Serial.println(compressorAmps);
  
  Serial.print("CNC: cool=");
  Serial.print(cncCoolant);
  Serial.print(", dir=");
  Serial.print(cncSpindleDirection);
  Serial.print(", speed=");
  Serial.print(cncSpindleSpeed);
  Serial.print(" (avg ");
  Serial.print(avgSpindleSpeed);
  Serial.println(")");

  rpm = analogToRpm(avgSpindleSpeed);
  Serial.print("RPM = ");
  Serial.print(rpm);
  Serial.println(" RPM");

  digitalWrite(pinAirA, airState);
  digitalWrite(pinAirB, airState);
  digitalWrite(pinAirC, airState);

  digitalWrite(pinSpindleOnOff, rpm > 0);

  //Serial.print("Average voltage is ");
  //Serial.println(1.0*totalVolts/countVolts);
  delay(100);

}
