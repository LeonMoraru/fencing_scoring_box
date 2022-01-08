//===========================================================================//
//                                                                           //
//  Desc:    Arduino Code to implement a fencing scoring apparatus           //
//  Dev:     Wnew                                                            //
//  Date:    Nov  2012                                                       //
//  Updated: Feb  2020 By RA  V20200712.01 - NanoEvery_LED                   //
//                                                                           //
//  Notes:   1. Basis of algorithm from digitalwestie on github. Thanks Mate //
//           2. Used uint8_t instead of int where possible to optimise       //
//           3. Set ADC prescaler to 16 faster ADC reads                     //
//                                                                           //
//  To do:   1. Could use shift reg on lights and mode LEDs to save pins     //
//           2. Implement short circuit LEDs (already provision for it)      //
//           3. Set up debug levels correctly                                //
//                                                                           //
//===========================================================================//

//============
// #libaries
//============
#include "LedControl.h"

LedControl lc = LedControl(12, 10, 11, 4);  // Pins: DIN,CLK,CS, # of Display connected

//============
// #defines
//============
// TODO: set up debug levels correctly
#define DEBUG 1
//#define TEST_LIGHTS       // turns on lights for a second on start up
//#define TEST_ADC_SPEED    // used to test sample rate of ADCs
//#define REPORT_TIMING     // prints timings over serial interface
#define BUZZERTIME 1000  // length of time the buzzer is kept on after a hit (ms)
#define LIGHTTIME 5000   // length of time the lights are kept on after a hit (ms)
#define BAUDRATE 57600   // baudrate of the serial debug interface

//============
// Pin Setup
//============
const uint8_t shortLEDA = 8;    // Short Circuit A Light
const uint8_t onTargetA = 9;    // On Target A Light
const uint8_t offTargetA = 10;  // Off Target A Light
const uint8_t offTargetB = 11;  // Off Target B Light
const uint8_t onTargetB = 12;   // On Target B Light
const uint8_t shortLEDB = 13;   // Short Circuit B Light

const uint8_t groundPinA = A0;  // Ground A pin - Analog
const uint8_t weaponPinA = A1;  // Weapon A pin - Analog
const uint8_t lamePinA = A2;    // Lame   A pin - Analog (Epee return path)
const uint8_t lamePinB = A3;    // Lame   B pin - Analog (Epee return path)
const uint8_t weaponPinB = A4;  // Weapon B pin - Analog
const uint8_t groundPinB = A5;  // Ground B pin - Analog

const uint8_t modePin = 2;             // Mode change button interrupt pin 0 (digital pin 2)
const uint8_t buzzerPin = 3;           // buzzer pin
const uint8_t modeLeds[] = {4, 5, 6};  // LED pins to indicate weapon mode selected {f e s}

//=========================
// values of analog reads
//=========================
int weaponA = 0;
int weaponB = 0;
int lameA = 0;
int lameB = 0;
int groundA = 0;
int groundB = 0;

//=======================
// depress and timeouts
//=======================
long depressAtime = 0;
long depressBtime = 0;
bool lockedOut = false;

//==========================
// Lockout & Depress Times
//==========================
// the lockout time between hits for foil is 300ms +/-25ms
// the minimum amount of time the tip needs to be depressed for foil 14ms +/-1ms
// the lockout time between hits for epee is 45ms +/-5ms (40ms -> 50ms)
// the minimum amount of time the tip needs to be depressed for epee 2ms
// the lockout time between hits for sabre is 120ms +/-10ms
// the minimum amount of time the tip needs to be depressed (in contact) for sabre 0.1ms -> 1ms
// These values are stored as micro seconds for more accuracy
//                         foil   epee   sabre
const long lockout[] = {300000, 45000, 170000};  // the lockout time between hits
const long depress[] = {14000, 2000, 1000};      // the minimum amount of time the tip needs to be depressed

//=================
// mode constants
//=================
const uint8_t FOIL_MODE = 0;
const uint8_t EPEE_MODE = 1;
const uint8_t SABRE_MODE = 2;

uint8_t currentMode = EPEE_MODE;

bool modeJustChangedFlag = false;

//=========
// states
//=========
bool depressedA = false;
bool depressedB = false;
bool hitOnTargA = false;
bool hitOffTargA = false;
bool hitOnTargB = false;
bool hitOffTargB = false;

uint8_t xhitOnTargA = LOW;
uint8_t xhitOffTargA = LOW;
uint8_t xhitOnTargB = LOW;
uint8_t xhitOffTargB = LOW;

//#ifdef TEST_ADC_SPEED
// long now;
// long loopCount = 0;
// bool done = false;
//#endif

// Put values in arrays
byte Complete[] =
    {
        B11111111,  // On frame
        B11111111,
        B11111111,
        B11111111,
        B11111111,
        B11111111,
        B11111111,
        B11111111};

byte Small[] =
    {
        B00000000,  // Square frame
        B00000000,
        B00111100,
        B00111100,
        B00111100,
        B00111100,
        B00000000,
        B00000000};

byte LEDFoil[] =
    {
        B00000000,  // Foil F
        B00000100,
        B00000100,
        B00011100,
        B00011100,
        B00000100,
        B00111100,
        B00000000,
};

byte LEDEpee[] =
    {
        B00000000,  // Epee E
        B00111100,
        B00000100,
        B00011100,
        B00011100,
        B00000100,
        B00111100,
        B00000000};

byte LEDSabre[] =
    {
        B00000000,  // Sabre S
        B00111100,
        B00100000,
        B00111100,
        B00000100,
        B00111100,
        B00000000,
        B00000000};
//================
// Configuration
//================
void setup() {
   // set the internal pullup resistor on modePin
   pinMode(modePin, INPUT_PULLUP);

   // add the interrupt to the mode pin (interrupt is pin 0)
   attachInterrupt(modePin, changeMode, RISING);
   pinMode(modeLeds[0], OUTPUT);
   pinMode(modeLeds[1], OUTPUT);
   pinMode(modeLeds[2], OUTPUT);

   // set the light pins to outputs
   // pinMode(offTargetA, OUTPUT);
   // pinMode(offTargetB, OUTPUT);
   // pinMode(onTargetA,  OUTPUT);
   // pinMode(onTargetB,  OUTPUT);
   // pinMode(shortLEDA,  OUTPUT);
   // pinMode(shortLEDB,  OUTPUT);
   pinMode(buzzerPin, OUTPUT);

   // digitalWrite(modeLeds[currentMode], HIGH);

   lc.shutdown(0, false);  // Wake up displays
   lc.shutdown(1, false);
   lc.shutdown(2, false);
   lc.shutdown(3, false);
   lc.setIntensity(0, 1);  // Set intensity levels
   lc.setIntensity(1, 1);
   lc.setIntensity(2, 1);
   lc.setIntensity(3, 1);
   lc.clearDisplay(0);  // Clear Displays
   lc.clearDisplay(1);
   lc.clearDisplay(2);
   lc.clearDisplay(3);

   testLights();
   setModeLeds();

   // this optimises the ADC to make the sampling rate quicker
   // adcOpt();

   Serial.begin(BAUDRATE);
   Serial.println("3 Weapon Scoring Box");
   Serial.println("====================");
   Serial.print("Mode : ");
   Serial.println(currentMode);

   resetValues();
}

//  Take values in Arrays and Display them
void LEDOnTargetA() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(0, i, Complete[i]);
   }
}

void LEDOnTargetB() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(3, i, Complete[i]);
   }
}

void LEDOffTargetA() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(1, i, Complete[i]);
   }
}

void LEDOffTargetB() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(2, i, Complete[i]);
   }
}

void LEDShortA() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(1, i, Small[i]);
   }
}

void LEDShortB() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(2, i, Small[i]);
   }
}

void ShowLEDFoil() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(1, i, LEDFoil[i]);
   }
}

void ShowLEDEpee() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(1, i, LEDEpee[i]);
   }
}

void ShowLEDSabre() {
   for (int i = 0; i < 8; i++) {
      lc.setRow(1, i, LEDSabre[i]);
   }
}

//=============
// ADC config
//=============
// void adcOpt() {

// the ADC only needs a couple of bits, the atmega is an 8 bit micro
// so sampling only 8 bits makes the values easy/quicker to process
// unfortunately this method only works on the Due.
// analogReadResolution(8);

// Data Input Disable Register
// disconnects the digital inputs from which ever ADC channels you are using
// an analog input will be float and cause the digital input to constantly
// toggle high and low, this creates noise near the ADC, and uses extra
// power Secondly, the digital input and associated DIDR switch have a
// capacitance associated with them which will slow down your input signal
// if you’re sampling a highly resistive load
// DIDR0 = 0x7F;

// set the prescaler for the ADCs to 16 this allowes the fastest sampling
// bitClear(ADCSRA, ADPS0);
// bitClear(ADCSRA, ADPS1);
// bitSet  (ADCSRA, ADPS2);

//}

//============
// Main Loop
//============
void loop() {
   // use a while as a main loop as the loop() has too much overhead for fast analogReads
   // we get a 3-4% speed up on the loop this way
   while (1) {
      checkIfModeChanged();
      // read analog pins
      weaponA = analogRead(weaponPinA);
      weaponB = analogRead(weaponPinB);
      lameA = analogRead(lamePinA);
      lameB = analogRead(lamePinB);
      // Serial.print  ("WeaponA : ");
      // Serial.println(weaponA);
      // Serial.print  ("WeaponB : ");
      // Serial.println(weaponB);
      // Serial.print  ("lameA : ");
      // Serial.println(lameA);
      // Serial.print  ("lameB : ");
      // Serial.println(lameB);
      signalHits();
      // delay(2000);
      if (currentMode == FOIL_MODE)
         foil();
      else if (currentMode == EPEE_MODE)
         epee();
      else if (currentMode == SABRE_MODE)
         sabre();

#ifdef TEST_ADC_SPEED
      if (loopCount == 0) {
         now = micros();
      }
      loopCount++;
      if ((micros() - now >= 1000000) && !done) {
         Serial.print(loopCount);
         Serial.println(" readings in 1 sec");
         done = true;
      }
#endif
   }
}

//=====================
// Mode pin interrupt
//=====================
void changeMode() {
   // set a flag to keep the time in the ISR to a min
   modeJustChangedFlag = true;
}

//============================
// Sets the correct mode led
//============================
void setModeLeds() {
   if (currentMode == FOIL_MODE) {
      ShowLEDFoil();
   } else if (currentMode == EPEE_MODE) {
      ShowLEDEpee();
   } else if (currentMode == SABRE_MODE) {
      ShowLEDSabre();
   }

   delay(2000);
   // Clear Displays
   for(int i = 0; i < 4; i++) {
      lc.clearDisplay(i);
   }
}

//========================
// Run when mode changed
//========================
void checkIfModeChanged() {
   if (modeJustChangedFlag) {
      if (digitalRead(modePin)) {
         if (currentMode == 2)
            currentMode = 0;
         else
            currentMode++;
      }
      setModeLeds();
#ifdef DEBUG
      Serial.print("Mode changed to: ");
      Serial.println(currentMode);
#endif
      modeJustChangedFlag = false;
   }
}

//===================
// Foil method per person
//===================
void foil_sub(bool &hitOnTarget, bool &hitOffTarget, bool &depressed, int &weaponAnalog, int &lameAnalog, long &depressedTime) {
   if (!hitOnTarget && !hitOffTarget) {  // ignore if it has already hit
      // off target
      if (900 < weaponAnalog && lameAnalog < 100) {
         if (!depressed) {
            depressedTime = micros();
            depressed = true;
         } else {
            if (depressedTime + depress[0] <= micros()) {
               hitOffTarget = true;
            }
         }
      } else {
         // on target
         if (300 < weaponAnalog && weaponAnalog < 600 && 300 < lameAnalog && lameAnalog < 600) {
            if (!depressed) {
               depressedTime = micros();
               depressed = true;
            } else {
               if (depressedTime + depress[0] <= micros()) {
                  hitOnTarget = true;
               }
            }
         } else {
            // reset these values if the depress time is short.
            depressedTime = 0;
            depressed = 0;
         }
      }
   }
}

//===================
// Main foil method
//===================
void foil() {
   long now = micros();
   if (((hitOnTargA || hitOffTargA) && depressAtime + lockout[0] < now) ||
       ((hitOnTargB || hitOffTargB) && depressBtime + lockout[0] < now)) {
      lockedOut = true;
   }

   foil_sub(hitOnTargA, hitOffTargA, depressedA, weaponA, lameB, depressAtime);
   foil_sub(hitOnTargB, hitOffTargB, depressedB, weaponB, lameA, depressBtime);
}

//===================
// Epee method per person
//===================
void epee_sub(bool &hitOnTarget, bool &depressed, int &weaponAnalog, int &lameAnalog, long &depressedTime) {
   //  no hit for A yet    && weapon depress    && opponent lame touched
   if (!hitOnTarget) {
      if (400 < weaponAnalog && weaponAnalog < 600 && 400 < lameAnalog && lameAnalog < 600) {
         if (!depressed) {
            depressedTime = micros();
            depressed = true;
         } else if (depressedTime + depress[1] <= micros()) {
            hitOnTarget = true;
         }
      } else if (depressed) {  // reset these values if the depress time is short.
         depressedTime = 0;
         depressed = 0;
      }
   }
}

//===================
// Main epee method
//===================
void epee() {
   long now = micros();
   if ((hitOnTargA && depressAtime + lockout[1] < now) || (hitOnTargB && depressBtime + lockout[1] < now)) {
      lockedOut = true;
   }

   epee_sub(hitOnTargA, depressedA, weaponA, lameB, depressAtime);
   epee_sub(hitOnTargB, depressedB, weaponB, lameA, depressBtime);
}

//===================
// Sabre method per person
//===================
void sabre_sub(bool &hitOnTarget, bool &depressed, int &weaponAnalog, int &lameAnalog, long &depressedTime) {
   if (!hitOnTarget) {  // ignore if A has already hit
      // on target
      if (300 < weaponAnalog && weaponAnalog < 450 && 300 < lameAnalog && lameAnalog < 600) {
         if (!depressed) {
            depressedTime = micros();
            depressed = true;
         } else if (depressedTime + depress[2] <= micros()) {
            hitOnTarget = true;
         }
      } else {
         // reset these values if the depress time is short.
         depressedTime = 0;
         depressed = 0;
      }
   }
}

//===================
// Main sabre method
//===================
void sabre() {
   long now = micros();
   if ((hitOnTargA && depressAtime + lockout[2] < now) || (hitOnTargB && depressBtime + lockout[2] < now)) {
      lockedOut = true;
   }

   sabre_sub(hitOnTargA, depressedA, weaponA, lameB, depressAtime);
   sabre_sub(hitOnTargB, depressedB, weaponB, lameA, depressBtime);
}

//==============
// Signal Hits
//==============
void signalHits() {
   // non time critical, this is run after a hit has been detected
   if (lockedOut) {
      xhitOnTargA = LOW;
      xhitOffTargA = LOW;
      xhitOnTargB = LOW;
      xhitOffTargB = LOW;
      if (hitOnTargA)
         LEDOnTargetA();

      if (hitOffTargA)
         LEDOffTargetA();

      if (hitOnTargB)
         LEDOnTargetB();

      if (hitOffTargB)
         LEDOffTargetB();

      digitalWrite(buzzerPin, LOW);
#ifdef DEBUG
      String serData = String("hitOnTargA  : ") + hitOnTargA + "\n" + "hitOffTargA : " + hitOffTargA + "\n" + "hitOffTargB : " + hitOffTargB + "\n" + "hitOnTargB  : " + hitOnTargB + "\n" + "Locked Out  : " + lockedOut + "\n";
      Serial.println(serData);
#endif
      resetValues();
   }
}

//======================
// Reset all variables
//======================
void resetValues() {
   delay(BUZZERTIME);  // wait before turning off the buzzer
   digitalWrite(buzzerPin, HIGH);
   delay(LIGHTTIME - BUZZERTIME);  // wait before turning off the lights
   lc.clearDisplay(0);             // Clear Displays
   lc.clearDisplay(1);
   lc.clearDisplay(2);
   lc.clearDisplay(3);

   lockedOut = false;
   depressAtime = 0;
   depressedA = false;
   depressBtime = 0;
   depressedB = false;

   hitOnTargA = false;
   hitOffTargA = false;
   hitOnTargB = false;
   hitOffTargB = false;

   delay(100);
}

//==============
// Test lights
//==============
void testLights() {
   digitalWrite(buzzerPin, HIGH);
   LEDOnTargetA();
   delay(250);
   LEDOffTargetA();
   delay(250);
   LEDOnTargetB();
   delay(250);
   LEDOffTargetB();
   delay(250);
   LEDShortA();
   delay(250);
   LEDShortB();
   delay(250);
   resetValues();
}
