/*

  Control code for driving a AMB/Kress FME-W DI Tool-changing spindle from GRBL.

  This is intended to run on an Arduino Mega that is wired up between
  the GRBL controller and the spindle.  It reads spindle speed,
  spindle direction, and coolant outputs from GRBL and outputs a
  number of distinct controls:

    - Spindle relay control (for powering the spindle down entirely).
    - Spindle speed control (0-5v PWM; will need converted to 0-10v.
      The AMB spindle reads 0v as 5k RPM and 10v as 25k RPM, while
      GRBL outputs 0v as off, 0.1v as minimum RPM and 5v as maximum
      RPM.
    - 3x pneumatic solenoid controls.  2 for driving the tool changer
      1 for use as an air assist/chip blower.
  
  In addition, it reads from several ACS712-based current meters,
  which are used to track the amount of current used by various parts
  of the control circuitry of the system.

  Finally, it drives a display, currently planned to be a 2.7" OLED
  from Adafruit via SPI.  The display will show the current RPM,
  current readings, and tool change status.

*/

// include the library code:
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1325.h>

// I/O pin definitions
const int oledCS=48; // Green wire
const int oledReset=47; // Orange wire
const int oledDC=49; // Blue wire

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

// RPM metrics.  v5000 is the value read on pinCncSpindleSpeed when
// GRBL is sending 5,000 RPM.  v25000 is the value read on
// pinCncSpindleSpeed when GRBL is sending 25,000 RPM.  Finally,
// anything under v_off is considered to be 'off'.
//
// There really needs to be a state machine here that limits
// transitions in and out of 'off' and blocks the spindle from
// spinning while the ATC is in operation.  Also, spindle speed is
// currently tracked using a decaying average, so we can average
// readings across several cycles.  The produced RPM needs to be a bit
// cleverer than this; it needs to stay fixed if the input voltage is
// close to the existing setpoint, but then skew faster once it's
// clearly out of bounds.  TODO(laird): review literature.
const double v5000=524.5;
const double v25000=643;
const double v_off=512;


// Spindle States.
const int stateSpindleOff=0;  // Can go to SpeedChanging or ToolChangeStart
const int stateSpindleSpeedChanging=1; // Can go to Off or Steady
const int stateSpindleSpeedSteady=2; // Can go to Changing
const int stateSpindleToolChangeStart=3; // Can go to ToolChanging
const int stateSpindleToolChanging=4; // Can go to ToolChangeWait
const int stateSpindleToolChangeWait=5; // Can go to Off
int stateSpindle = stateSpindleOff;

Adafruit_SSD1325 display(oledDC, oledReset,oledCS);

// DisplayMessages are a timestamp and a pointer to a constant string.  The string will not be freed.
struct DisplayMessage {
  unsigned long timestamp;
  char *message;
  // Add a priority?
};

// set timestamp=0 once free.
const int messagelimit=10;
struct DisplayMessage messages[messagelimit];

void DisplayMessage(char *message) {
  unsigned long timestamp = millis();

  for (int i=0; i<messagelimit; i++) {
    if (messages[i].timestamp==0) {
      messages[i].timestamp = timestamp;
      messages[i].message=message;
      return;
    }
  }
  // Failed to find a free slot.
  Serial.println(">>> displaymessage failed=true");
}

void CleanMessages() {
  unsigned long oldest_timestamp = millis() - 10000;
  
  for (int i=0; i<messagelimit; i++) {
    if (messages[i].timestamp < oldest_timestamp) {
      messages[i].timestamp=0;
      messages[i].message=0;
    }
  }
}

char *GetNewestMessage() {
  unsigned long newest_timestamp = 0;
  char *newest_message = 0;

  for (int i=0; i<messagelimit; i++) {
    if (messages[i].timestamp > newest_timestamp) {
      newest_timestamp=messages[i].timestamp;
      newest_message=messages[i].message;
    }
  }

  return newest_message;
}

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
// overflow cases.  So: double for now, and it can move to longs if
// there's a demonstrable performance problem.
//
// Note: float==double on Arduino.
double rawToAmps(int raw, int acs712_amps) {
  int mVperAmp = 185;
  int ACSoffset = 2582;  // Averaged across several devices; supposed be 2500.
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
  voltage = raw / 1024.0 * 5000;
  amps = (voltage - ACSoffset) / mVperAmp;
  
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


int analogToRpm(double v){
  if (v < v_off)
    return 0; // Off
      
  return ((v-v5000)/((v25000-v5000)/20.0)+5)*1000;
}

void setup() {
  // Set up logging over USB
  Serial.begin(115200);
  Serial.println(">>> booting");

  // Set up pins
  pinMode(pinSpindleOnOff, OUTPUT);
  pinMode(pinAirA, OUTPUT);
  pinMode(pinAirB, OUTPUT);
  pinMode(pinAirC, OUTPUT);

  display.begin();
  display.display();
  delay(1000);
  display.clearDisplay();
}

double dcAmps, spindleAmps, vacAmps, compressorAmps;
double avgSpindleSpeed;

void readAmps() {
  double new_dcAmps, new_spindleAmps, new_vacAmps, new_compressorAmps;

  // Read current meters
  new_dcAmps = rawToAmps(analogRead(pinDcAmps), 20);
  new_spindleAmps = rawToAmps(analogRead(pinSpindleAmps), 5);
  new_vacAmps = rawToAmps(analogRead(pinVacAmps), 20);
  new_compressorAmps = rawToAmps(analogRead(pinCompressorAmps), 20);

  dcAmps = decayingAverage(new_dcAmps, dcAmps, 0.2);
  spindleAmps = decayingAverage(new_spindleAmps, spindleAmps, 0.2);
  vacAmps = decayingAverage(new_vacAmps, vacAmps, 0.2);
  compressorAmps = decayingAverage(new_compressorAmps, compressorAmps, 0.2);
  Serial.print(">>> current dc=");
  Serial.print(dcAmps);
  Serial.print(" spindle=");
  Serial.print(spindleAmps);
  Serial.print(" vac=");
  Serial.print(vacAmps);
  Serial.print(" compressor=");
  Serial.println(compressorAmps);
}

int cncCoolant, cncSpindleDirection, cncSpindleSpeed;
void readGrbl() {
  cncSpindleSpeed=analogRead(pinCncSpindleSpeed);
  cncSpindleDirection=digitalRead(pinCncSpindleDirection);
  cncCoolant=digitalRead(pinCncCoolant);
  avgSpindleSpeed=decayingAverage(cncSpindleSpeed, avgSpindleSpeed, 0.2);

  Serial.print(">>> grbl cool=");
  Serial.print(cncCoolant);
  Serial.print(" dir=");
  Serial.print(cncSpindleDirection);
  Serial.print(" rawSpeed=");
  Serial.print(cncSpindleSpeed);
  Serial.print(" avgSpeed=");
  Serial.println(avgSpindleSpeed);
}

long doneToolChangeStart, doneToolChangeWait;

void loop() {
  int rpm, effectiveRpm;
  int wantToolChange;

  // Generic serial logging format: start with '>>> ', then a word
  // that describes which type of logging, then 0 or more name=value
  // pairs, followed by a newline.
  Serial.print(">>> starting ms=");
  Serial.print(millis());
  Serial.print(" statespindle=");
  Serial.println(stateSpindle);

  readAmps();
  readGrbl();

  wantToolChange=cncCoolant==0 && cncSpindleDirection==1;

  rpm = analogToRpm(avgSpindleSpeed);

  switch (stateSpindle) {
  case stateSpindleOff: 
    effectiveRpm=0;
    if (wantToolChange) {
      Serial.println(">>> spindlestate old=off new=toolchangestart");
      DisplayMessage("Starting tool change");
      stateSpindle=stateSpindleToolChangeStart;
      doneToolChangeStart = millis() + 5000; // 5 second wait to make sure the spindle has spun down completely.
    } else if (rpm > 0) {
      Serial.println(">>> spindlestate old=off new=speedchanging");
      DisplayMessage("Spindle starting");
      stateSpindle=stateSpindleSpeedChanging;
    }
    break;
  case stateSpindleSpeedChanging:
    effectiveRpm=rpm;
    if (rpm > 0) {
      Serial.println(">>> spindlestate old=speedchanging new=speedsteady");
      stateSpindle=stateSpindleSpeedSteady;
    } else {
      Serial.println(">>> spindlestate old=speedchanging new=off");
      DisplayMessage("Spindle stoping");
      stateSpindle=stateSpindleOff;
    }
    break;
  case stateSpindleSpeedSteady: 
    effectiveRpm=rpm;
    Serial.println(">>> spindlestate old=speedsteady new=speedchanging");
    stateSpindle=stateSpindleSpeedChanging;
    break;
  case stateSpindleToolChangeStart:
    if (millis() > doneToolChangeStart) {
      Serial.println(">>> spindlestate old=toolchangestart new=toolchanging");
      DisplayMessage("Unlocking tool");
      stateSpindle=stateSpindleToolChanging;
    }
    break;
  case stateSpindleToolChanging:
    digitalWrite(pinAirA, 1);  // Unlock
    digitalWrite(pinAirB, 1);  // TODO(laird): pulse 500ms on, 500ms off, 500ms on, 500ms off
    
    if (!wantToolChange) {
      Serial.println(">>> spindlestate old=toolchanging new=toolchangewait");
      DisplayMessage("Locking tool");
      doneToolChangeWait = millis() + 5000;  // 5 second wait before unlocking spindle.
      stateSpindle = stateSpindleToolChangeWait;
    }
    break;
  case stateSpindleToolChangeWait:
    digitalWrite(pinAirA, 0);  // lock
    digitalWrite(pinAirB, 0);  // lock
    if (millis() > doneToolChangeWait) {
      Serial.println(">>> spindlestate old=toolchangewait new=off");
      DisplayMessage("Tool change complete");
      stateSpindle=stateSpindleOff;
    }
    break;
  }
  
  //lcd.clear();
  //lcd.print("DC: ");
  //lcd.print(dcAmps, 3);
  //lcd.print("A");

  //lcd.setCursor(0, 1);
  //lcd.print(millis() / 1000);

  char buf[40];

  display.clearDisplay();  // Probably slow
  display.setTextColor(WHITE); 
  display.setTextSize(2);
  display.setCursor(0,0);
  int sa = spindleAmps * 10 + 0.5;
  int sa_fullamps = sa/10;
  int sa_fractionamps = sa-sa_fullamps*10;
  
  display.print(sa_fullamps);
  display.print(".");
  display.print(sa_fractionamps);
  display.setTextSize(1);
  display.print("A");
  
  display.setCursor(128-78,0); // 6 pixel wide font.  "RPM" + 5 double-width characters = 13*6=78.
  display.setTextSize(2);
  sprintf(buf,"%5d", effectiveRpm);
  display.print(buf);
  display.setTextSize(1);
  display.println("RPM");
  display.setTextSize(1);
  display.println("");

  char *message = GetNewestMessage();

  if(message) {
    display.setTextSize(1);
    display.setCursor(0,64-8);
    display.println(message);
  }

  display.display();
  CleanMessages();
  
  Serial.print("RPM = ");
  Serial.print(effectiveRpm);
  Serial.println(" RPM");

  //digitalWrite(pinAirA, airState);
  //digitalWrite(pinAirB, airState);
  //digitalWrite(pinAirC, airState);

  digitalWrite(pinSpindleOnOff, effectiveRpm > 0);

  delay(100);

}
