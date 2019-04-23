/*

  Copyright 2019 Google LLC

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

     https://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

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


  Tool change procedure:
  - Stop spindle: M3 S0  (S0 is good enough)
  - Move to wherever you want the tool to live.  An open slot in a
    tool rack would be ideal, and also cheaper than most of the
    alternatives. 
  - Unlock and eject the tool: M4 S1500 (M4 is "spin backwards", S1500
    is spin at 1500 RPM.  The FME-W can't do either of these things
  - The display will say "Unlocking tool" and wait for 5 seconds.
  - Then it will eject the tool, leaving the spindle unlocked.
  - Next, move to the location of the next tool.
  - To lock the spindle: M3 S0.
  - Once this occurs, the tool will be locked, but this code will
    refuse to start spinning for 5 seconds.  Use that time to move the
    spindle up away from the tool rack.
  - To start spinning, use M3 S10000 (or whatever RPM), just like
    normal.  Your CAM preprocessor almost certainly put one of these
    into your gcode for you.

*/

// include the library code:
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1325.h>
#include <eRCaGuy_Timer2_Counter.h>

// I/O pin definitions
const int oledCS=48; // Green wire
const int oledReset=47; // Orange wire
const int oledDC=49; // Blue wire

const int pinDcAmps=A0;
const int pinSpindleAmps=A1;
const int pinCompressorAmps=A2;
const int pinVacAmps=A3;
const int pinAirPressure=A5;

const int pinCncSpindleSpeed=2;
const int intCncSpindleSpeed=0;  // Interrupt bound to pin 2.

const int pinCncSpindleDirection=4; // M3 on GRBL sets this to 0, M4 sets to 1.
const int pinCncCoolant=5;  // M8 on GRBL sets this to 0, M9 sets this to 1.
const int pinSpindleOnOff=6;
const int pinAirBlower=7;
const int pinAirEject=8;
const int pinAirLock=9;
const int pinFaultOut=18;

const int pinStepperFault1=14;  // X
const int pinStepperFault2=15;  // Y1
const int pinStepperFault3=16;  // Y2
const int pinStepperFault4=17;  // Z

const int pinSpindleSpeedOut=44;

// Spindle States.
const int stateSpindleOff=0;  // Can go to Running or ToolChangeStart
const int stateSpindleRunning=1; // Can go to Off
const int stateSpindleToolChangeStart=2; // Will go to ToolChangeEject
const int stateSpindleToolChangeEject=3; // Will go to ToolChanging
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
  amps = fabs(voltage - ACSoffset) / mVperAmp;
  
  return amps;
}

double decayingAverage(double a, double b, double fraction) {
  return a * fraction + b * (1 - fraction);
}

// Spindle PWM reading code
// Mostly from http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
//
// Using 'timer2' from eRC...guy, with 0.5 usec resolution, because
// millis() has 4us resolution, which is ~100ish RPM.
//
// Observed values, with $31=5000 and $32=25000 (min and max spindle speeds).
//
// - s0: Value doesn't update, because it never rises.
// - s5000: 23
// - s10000: ~523
// - s20000: ~1540
// - s24000: ~1946
// - s24999: ~2043
// - s25000: doesn't update, because it never falls.
//
// So, we need to handle s0 (off, no PWM) and s25000 (on, no PWM)
// specially.  We'll do this via 'pwmSeen'.  We'll set it to 2 in each
// interrupt handler and then decrement it in loop().  If it hits 0,
// then we'll set RPMs to either 0 or 25000 based on digitalRead() of
// the spindle pin.


volatile long pwmSpindleValue=0;
volatile long pwmSpindleRiseTime=0;
volatile int pwmSeen=0;
const unsigned int spindleMin=1000;
const unsigned int spindleMax=25000;

void spindlePWMRising() {
  attachInterrupt(intCncSpindleSpeed, spindlePWMFalling, FALLING);
  pwmSeen=5;
  pwmSpindleRiseTime=timer2.get_count(); //micros();
}

void spindlePWMFalling() {
  attachInterrupt(intCncSpindleSpeed, spindlePWMRising, RISING);
  pwmSeen=5;
  pwmSpindleValue=timer2.get_count()-pwmSpindleRiseTime;  // was micros()
}

unsigned int readSpindleSpeed() {
  unsigned int rpm;
  
  if(--pwmSeen<=0) {
    pwmSeen=0;  // Don't want to to go ouboundedly negative.
    // there's no PWM; it's either off on on.
    rpm = digitalRead(pinCncSpindleSpeed) ? spindleMax : 0;
  } else {
    // We've seen a PWM interrupt during the last cycle
    double v = pwmSpindleValue; // might change under us

    const int min_value = 58;  // For $31=500 and spindleMin=1000.
    const int max_value = 2042;
    
    Serial.print(">>> rpm val=");
    Serial.print(v);

    v -= min_value;  // minimum observed value, ish
    v /= (max_value-min_value); // observed range

    if (v<0)
      v=0;

    if (v>1)
      v=1;

    Serial.print(" rpm=");
    Serial.println(rpm);
    rpm = (spindleMax-spindleMin)*v+spindleMin;
  }

  rpm = (rpm + 5) / 10;
  rpm = rpm * 10;

  return rpm;
}


int rpmToAnalog(double rpm) {
  double r, s;
  
  r = rpm-5000 + 2550; // 2550 offset measured to get 5000 RPM working accurately.
  if (r<0)
    r=0;

  //s=r/20000;
  s=r/22133; // at s20000 (r=17550 with offset above), spindle
	     // measures 21600 RPM.  So, asking for an increase of
	     // 15000 gives us 16600 extra RPM.  Therefore, increasing
	     // the divisor to 16.6/15*20000=22133.

  if (s>1)
    s=1;
  
  return s*255;
}

void setup() {
  // Set up logging over USB
  Serial.begin(115200);
  Serial.println(">>> booting");

  timer2.setup();

  // Set up PWM spindle interrupt
  attachInterrupt(intCncSpindleSpeed, spindlePWMFalling, FALLING);

  // Set up pins
  pinMode(pinSpindleOnOff, OUTPUT);
  pinMode(pinAirLock, OUTPUT);
  pinMode(pinAirEject, OUTPUT);
  pinMode(pinAirBlower, OUTPUT);
  pinMode(pinSpindleSpeedOut, OUTPUT);
  pinMode(pinFaultOut, OUTPUT);

  pinMode(pinStepperFault1, INPUT_PULLUP);
  pinMode(pinStepperFault2, INPUT_PULLUP);
  pinMode(pinStepperFault3, INPUT_PULLUP);
  pinMode(pinStepperFault4, INPUT_PULLUP);

  display.begin();
  display.clearDisplay();
  display.display();
  delay(1000);
}

double dcAmps, spindleAmps, vacAmps, compressorAmps;

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
  cncSpindleDirection=digitalRead(pinCncSpindleDirection);
  cncCoolant=digitalRead(pinCncCoolant);

  Serial.print(">>> grbl cool=");
  Serial.print(cncCoolant);
  Serial.print(" dir=");
  Serial.println(cncSpindleDirection);
}

double airPressure; // in PSI
const double zeroAirPressure = 496;
const double fullAirPressure = 784;

// Read from the air pressure sensor and set airPressure to the pressure in PSI.
//
// Measured values:
// 0 PSI: 496
// 100 PSI: 784
//
void readPressure(void) {
  int raw;
  
  raw = analogRead(pinAirPressure);
  airPressure = (raw - zeroAirPressure)/(fullAirPressure-zeroAirPressure)*100;
  Serial.print(">>> pressure raw=");
  Serial.print(raw);
  Serial.print(" psi=");
  Serial.println(airPressure);
}

int faultStepper1, faultStepper2, faultStepper3, faultStepper4;
int faultReceived;
void readFault(void) {
  int newFaultReceived;

  // These are all inverted.
  faultStepper1=!digitalRead(pinStepperFault1);
  faultStepper2=!digitalRead(pinStepperFault2);
  faultStepper3=!digitalRead(pinStepperFault3);
  faultStepper4=!digitalRead(pinStepperFault4);

  newFaultReceived = faultStepper1||faultStepper2||faultStepper3||faultStepper4;
  faultReceived = newFaultReceived;  // Not sure if it should latch onto true and stay there or not.

  if (newFaultReceived) {
    if (faultStepper1) {
      DisplayMessage("Fault on X Stepper");
    } else if (faultStepper2) {
      DisplayMessage("Fault on Y1 Stepper");
    } else if (faultStepper3) {
      DisplayMessage("Fault on Y2 Stepper");
    } else if (faultStepper4) {
      DisplayMessage("Fault on Z Stepper");
    } else {
      DisplayMessage("Unknown Stepper Fault");
    }
  }

  Serial.print(">>> fault received=");
  Serial.print(faultReceived);
  Serial.print(" stepper1=");
  Serial.print(faultStepper1);
  Serial.print(" stepper2=");
  Serial.print(faultStepper2);
  Serial.print(" stepper3=");
  Serial.print(faultStepper3);
  Serial.print(" stepper4=");
  Serial.println(faultStepper4);
}

long doneToolChangeStart, doneToolChangeWait;
long doneEjectPhase;
int ejectPhase;

void loop() {
  unsigned int rawRpm, rpm, effectiveRpm;
  int wantToolChange, wantCoolant, enableBlower;

  // Generic serial logging format: start with '>>> ', then a word
  // that describes which type of logging, then 0 or more name=value
  // pairs, followed by a newline.
  Serial.print(">>> starting ms=");
  Serial.print(millis());
  Serial.print(" statespindle=");
  Serial.println(stateSpindle);

  readAmps();
  readPressure();
  readGrbl();
  readFault();

  wantToolChange = cncSpindleDirection==1;
  wantCoolant = cncCoolant==0;
  enableBlower = 0;

  rpm = readSpindleSpeed();

  switch (stateSpindle) {
  case stateSpindleOff: 
    effectiveRpm=0;
    if (wantToolChange && rpm > 1000 && rpm < 2000) {
      Serial.println(">>> spindlestate old=off new=toolchangestart");
      DisplayMessage("Starting tool change");
      stateSpindle=stateSpindleToolChangeStart;
      doneToolChangeStart = millis() + 5000; // 5 second wait to make sure the spindle has spun down completely.
    } else if (rpm >= 4900) {
      Serial.println(">>> spindlestate old=off new=spindlerunning");
      DisplayMessage("Spindle starting");
      stateSpindle=stateSpindleRunning;
    }
    break;
  case stateSpindleRunning: 
    effectiveRpm=rpm;
    if (rpm < 4000) {
      Serial.println(">>> spindlestate old=running new=off");
      DisplayMessage("Spindle stopping");
      stateSpindle=stateSpindleOff;
    } else {
      enableBlower=wantCoolant;
    }
    break;
  case stateSpindleToolChangeStart:
    if (millis() > doneToolChangeStart) {
      Serial.println(">>> spindlestate old=toolchangestart new=toolchangeeject");
      DisplayMessage("Ejecting tool");
      stateSpindle=stateSpindleToolChangeEject;
      ejectPhase=0;
    }
    break;
  case stateSpindleToolChangeEject:
    // Eject the tool.  500ms on, 500ms off, 500ms on, 500ms off.
    // Phase 0 is starting.  Phase 1 and 3 have the eject air on.  Phases 2 and 4 have it off.
    if (ejectPhase==0) {
      ejectPhase=1;
      doneEjectPhase=millis()+500;
      digitalWrite(pinAirLock, 1);  // Unlock
      digitalWrite(pinAirEject, 1);
    } else if (millis() > doneEjectPhase) {
      ejectPhase++;
      Serial.print(">>> ejectstate phase=");
      Serial.println(ejectPhase);
      doneEjectPhase=millis()+500;
      if (ejectPhase>=4) {
	Serial.println(">>> spindlestate old=toolchangeeject new=toolchanging");
	DisplayMessage("Tool unlocked");
	stateSpindle=stateSpindleToolChanging;
	digitalWrite(pinAirEject, 0);
	break;
      }
      digitalWrite(pinAirEject, ejectPhase&1);  // Eject on odd numbered phases.
    }
    break;
  case stateSpindleToolChanging:
    digitalWrite(pinAirLock, 1);
    digitalWrite(pinAirEject, 0);
    
    if (!wantToolChange) {
      Serial.println(">>> spindlestate old=toolchanging new=toolchangewait");
      DisplayMessage("Locking tool");
      doneToolChangeWait = millis() + 5000;  // 5 second wait before unlocking spindle.
      stateSpindle = stateSpindleToolChangeWait;
    }
    break;
  case stateSpindleToolChangeWait:
    digitalWrite(pinAirLock, 0);  // lock
    digitalWrite(pinAirEject, 0);  // lock
    if (millis() > doneToolChangeWait) {
      Serial.println(">>> spindlestate old=toolchangewait new=off");
      DisplayMessage("Tool change complete");
      stateSpindle=stateSpindleOff;
    }
    break;
  }
  
  digitalWrite(pinAirBlower, enableBlower);
  digitalWrite(pinFaultOut, !faultReceived);  // actually backwards.
  
  char buf[40];

  display.clearDisplay();  // Probably slow
  display.setTextColor(WHITE); 
  display.setTextSize(2);
  display.setCursor(0,0);
  int sa = abs(spindleAmps * 10 + 0.5);
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
  
  int analogRpm = rpmToAnalog(effectiveRpm);

  digitalWrite(pinSpindleOnOff, effectiveRpm > 0);

  analogWrite(pinSpindleSpeedOut, rpmToAnalog(effectiveRpm));
  
  delay(100);

}
