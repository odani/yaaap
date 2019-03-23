/*
   YAAAP Yet Another Arduino AutoPilot
   yaaap.ino
   2016 Philippe Leclercq

   Heading is given by the 9dof IMU MPU9250 using the RTIMUlib for Arduino by Richards-Tech.
   RTIMUlib filter is especially effective to keep a steady course when the heel angle varies.
   I reduced the impact of magnetometer instability with a reduced SLERP value: fusion.setSlerpPower(0.001)
   The tiller actuator is the old cylinder from the original Autohelm 2000 tillerPilot (with Omron motor 2332 12V)
   Typical current for this motor is 1.6A. The cheap TB6612FNG can drive this motor with the 2 channels in parrallel (2A typical to 6A peak).
   It can be replaced by a more powerfull driver like IBT_2 with BTN7970B.
   I dropped the usage of PWM with my old small motor. Minimal pulse is 10ms, increasing to continuous depending
   on the computed command(PID).
   There is no device to read the tiller position. Current to the motor is controlled to detect end of course or overload (ACS712).
   The interface is minimal, made of 3 buttons using OneButtons library https://github.com/mathertel/OneButton
   The LCD display is an I2C version.

   Instructions
	The device is fixed behind the tiller facing the route.
 	Button 1 on the left: sarboard/less fonctions
	button 2 on center: , select/run-standby fonctions
	Button 3 on the right: port/plus fonctions

	On startup, after initialization, the device is on standby, bearing to the current direction.
	The display shows the status(Sby/Run), bearing, heading and error (difference to baering).

	To run, click once on button 2.

	During run, click on button 1 or 3 increases (resp. decrease) 1 deg of bearing.
				long press on button 1 or 3 increases (resp. decreases) 20 deg of bearing
				long press on button 2 resets bearing to current heading
				long press on button 1 and 2 (starboard and select) to tack to starboard
				long press on button 3 and 2 (port and select) to tack to port
				double click on button 2 to stop motor and standby

	During standby,	long press button 1 push the tiller
				long press on button 2 resets bearing to current heading
				long press button 3 pull the tiller
				long press button 1 and 3 to enter setup

				On setup first level,	click 1(-) for dimmed backlight
							click 3(+) for full backlight
							click 2(sel) to enter setup menu
				On setup menu,		click 1(-) to go to previous menu item
							click 3(+) to go to next menu item
							click 2(sel) to select menu item
				On setup item,		click 1(-) to decrease value if applicable
						        click 3(+) to increase value if applicable
							click 2(sel) to validate entry/value

	To calibrate the compass, chose the menu item, draw 8 with the device. The display shows the offsets.
	Click button 2 when OK.
	Important: go to the saving menu item and validate to write calibration and other parameters to EEPROM.

   */

#define DEBUG 10
#define SERIALDEBUG 5
//#define SERVO 1
#define TACKANGLE 100 // In degrees
#define OVERLOADCURRENT 6000 // milliAmps
#define OVERLOADDELAY 5000 // millisec
// motor definitions
//#define MOTORDRIVER 1 // BTN7970B
#define MOTORDRIVER 2 // TB6612FNG

#if MOTORDRIVER == 1 // BTN7970B
#define RPWM 3
#define LPWM 4
#define R_EN 5
#define L_EN 6
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
#define A_PWM 3
#define A_IN2 4
#define A_IN1 5
#define STBY 6
#define B_IN1 7
#define B_IN2 8
#define B_PWM 9
#endif
#define ACS712PIN A0

#include <Wire.h>
#include "LSM303.h"
#include "OneButton.h"
#include "I2Cdev.h" // this may be for previous compass?
#include "CalLib.h" // for previous compass but still using callib_data struct


#include <Adafruit_SSD1306.h> // for oled
#include <Adafruit_GFX.h> // for oled

#define LED_PIN 13
unsigned long ledStart = 0;

// Setup OneButtons on pins A1, A2, A3
OneButton button1(A1, true);
OneButton button2(A2, true);
OneButton button3(A3, true);

// Setup for LSM303D compass
LSM303 compass;

// OLED display TWI address
#define OLED_ADDR   0x3C
Adafruit_SSD1306 display(-1);

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

long refVoltage = 2490; //mV for ACS712

float cmd = 0;
bool standby = true;
bool launchTack = false;
byte deadband = 4;
float heading = 0, bearing = 0, headingError = 0;
float previousError = 0, deltaError = 0;
//float deltaErrorDeriv = 0, previousDeltaError = 0;
//int prevLoop = 0;
unsigned long previousTime = 0, deltaTime;
bool setupFonctions = false;
char keyPressed = ' ';
unsigned long prevDisplay = 0, prevCommand = 0;
#define COMMAND_INTERVAL   200   // interval between tiller command calculations
#define DISPLAY_INTERVAL  1000   // interval between heading displays

CALLIB_DATA params;              // the calibration data and other EEPROM params

int Kp = 3, Kd = 30; // proportional, derivative1 coefficients (tiller position do the integral/sum term)

#ifdef SERVO // For development and code test only
#include <Servo.h>
Servo myservo;
int servoAngle = 90;
#endif

/***********************************************************
   Subroutines
 ***********************************************************
*/
int sign(int val) { // declare the missing sign() fn
  return (val > 0) - (val < 0);
}

/*
// currently not using this 11/3/18
// this is for converting numbers to a string
// https://www.quora.com/Why-is-char-buffer-15-used-in-this-program
void printJustified(int value) {
  char buffer[7];
  sprintf(buffer, "%4d", value);
  //lcd.print(buffer);
}
*/
float map360(float deg) {
  if (deg < 0) deg += 360;
  else if (deg > 360) deg -= 360;
  return deg;
}
void beep(bool on) { // No sound, only a flashing led
  if (on) {
    digitalWrite(LED_PIN, true);
    ledStart = millis();
  }
  else if ((millis() - ledStart) > 100 ) {
    ledStart = 0; // End of beeb/light. Reset delay
    //   digitalWrite(LED_PIN, false);
  }
}

/***********************************************************
   Tiller/motor management
***********************************************************

*/
byte pulseState = LOW;
unsigned long pulseCurrentMillis;
unsigned long pulseStartMillis;
int tillerCurrent = 0;
int overloadDirection = 0;
unsigned long overloadStart = 0;

void tillerStandby(boolean state) {
  if (state == true) { // stop mode
#if MOTORDRIVER == 1 // BTN7970B
    analogWrite(RPWM, LOW);
    analogWrite(LPWM, LOW);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
    digitalWrite(A_PWM, LOW);
    digitalWrite(B_PWM, LOW);
#endif
    pulseState = LOW;
    digitalWrite(LED_PIN, false);
  }
}

void tillerInit() {
  // Init over current protection device (ACS712PIN)
  //  Serial.print("estimating avg. quiscent voltage:");
  //read X samples to stabilize value
  static const int nbSamples = 200;
  for (int i = 0; i < nbSamples; i++) {
    refVoltage += analogRead(ACS712PIN);
    delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
  }
  refVoltage /= nbSamples;
#if SERIALDEBUG == 1
  Serial.print(map(refVoltage, 0, 1023, 0, 5000)); Serial.println(" mV");
#endif
#if MOTORDRIVER == 1 // BTN7970B
  analogWrite(RPWM, LOW);
  analogWrite(LPWM, LOW);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
  pinMode(STBY, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_PWM, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);
  digitalWrite(STBY, HIGH);
#endif
  pulseStartMillis = millis();

  tillerStandby(true);
}

bool tillerOverload(int commandDirection) {
  const int sensitivity = 100;//change this to 100 for ACS712PIN-20A or to 66 for ACS712PIN-30A
  bool overload = false;
  if (overloadStart) {
    if (overloadDirection == commandDirection) {       // If same direction than last detected overload
      if ((millis() - overloadStart) < OVERLOADDELAY )  // wait delay
        overload = true;
      else {
        overloadStart = 0; // End of pause. Reset delay
      }
      return overload;
    }
    else {
      overloadStart = 0; // Direction has changed. Reset delay
    }
  }
  int v = 0;
  //read some samples to stabilize value
  for (int i = 0; i < 10; i++) {
    v = v + analogRead(ACS712PIN);
    delay(1);
  }
  v /= 10;
  #if SERIALDEBUG==5
    Serial.print("v: ");
    Serial.println(v);
  #endif

  v -=  refVoltage;

  // this is giving 3.9 Amps for my motor in the TP32
  tillerCurrent = v * 5000 / 1023 / sensitivity * 1000; // to mA
  if (abs(tillerCurrent) > OVERLOADCURRENT) {
    overload = true;
    overloadStart = millis();
    overloadDirection = commandDirection;
    tillerStandby(true);

    display.clearDisplay();
    display.setCursor(5,5);
    display.print("Overload ");
    display.print(tillerCurrent);
    display.display();
  }
  else {
    overloadDirection = 0;
  }
  return overload;
}

void tillerPush() {
#if MOTORDRIVER == 1 // BTN7970B
  analogWrite(LPWM, HIGH);
  analogWrite(RPWM, LOW);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, HIGH);
  digitalWrite(B_IN1, LOW);
  digitalWrite(B_IN2, HIGH);
#endif
#ifdef SERVO
  servoAngle += 1;
  if (servoAngle > 180) servoAngle = 180;
  myservo.write(servoAngle);
#endif
}

void tillerPull() {
#if MOTORDRIVER == 1 // BTN7970B
  analogWrite(LPWM, LOW);
  analogWrite(RPWM, HIGH);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
  digitalWrite(A_IN1, HIGH);
  digitalWrite(A_IN2, LOW);
  digitalWrite(B_IN1, HIGH);
  digitalWrite(B_IN2, LOW);
#endif
#ifdef SERVO
  servoAngle -= 1;
  if (servoAngle < 0) servoAngle = 0;
  myservo.write(servoAngle);
#endif
}

void tillerCommand(int tillerCmd) {

  // Current / motor overload / end of course control
  if (tillerOverload(sign(tillerCmd)))
    tillerCmd = 0;

#if DEBUG == 10
  display.clearDisplay();
  display.setCursor(5,5);
  display.print("motor ");
  display.print(tillerCmd);
  display.display();
#endif

  if (tillerCmd == 0) {
    tillerStandby(true);
    pulseStartMillis = millis();
    return;
  }

  pulseCurrentMillis = millis();
  int pulseWidth = 20 + abs(tillerCmd) * 18 / 10; // Higher command, longer pulse (20 to 200ms)
  if ((pulseCurrentMillis - pulseStartMillis) <= pulseWidth) {
    pulseState = HIGH;
    digitalWrite(LED_PIN, true);
  }
  else {
    if (abs(tillerCmd) < 100) { // 100 => continuous 
      tillerCmd = 0;
      pulseState = LOW;
      digitalWrite(LED_PIN, false);
      //     Serial.print("pulseW="); Serial.print(pulseWidth); Serial.print(" "); Serial.print(pulseCurrentMillis); Serial.print(" - "); Serial.print(pulseStartMillis); Serial.println(" ");
#if DEBUG == 3
  display.clearDisplay()
  display.setCursor(5,5);
  display.print(tillerCmd);
  display.print(" ");
  //printJustified(pulseState);
  display.print(pulseState);
  //printJustified(pulseWidth);
  display.display();
#endif
    }
    pulseStartMillis = millis();
  }

  if (tillerCmd > 0)
    tillerPull();
  else
    tillerPush();

#if MOTORDRIVER == 1 // BTN7970B
  digitalWrite(R_EN, pulseState);
  digitalWrite(L_EN, pulseState);
#elif MOTORDRIVER == 2 // TB6612FNG // Two channels in parallel for a 2Amp motor
  digitalWrite(A_PWM, pulseState);
  digitalWrite(B_PWM, pulseState);
#endif
}

/***********************************************************
   Compute tiller command
 ***********************************************************
*/
int computeCmd() {
  headingError = heading - bearing;
  // deadband saved to eeprom, is the heading error that is allowed before course correction initiates
  if (abs(headingError) < deadband / 2) {
    headingError = 0;
    cmd = 0;
  }
  else {
    if (abs(headingError) > 180) {
      if (bearing > heading)
        headingError += 360;
      else
        headingError -= 360;
    }

    unsigned long now = millis();
    deltaTime = now - previousTime;
    previousTime = now;
    deltaError = headingError - previousError;
    previousError = headingError;
  // deltaErrorDeriv = deltaError - previousDeltaError;
    //previousDeltaError = deltaError;

    cmd = Kp * headingError + (Kd * deltaError) * 1000 / deltaTime; // by time in sec
    if (cmd > 100) cmd = 100;
    if (cmd < -100) cmd = -100;
  }
#if DEBUG == 2
  //printJustified((int)(Kp*headingError));
  //printJustified((int)(Kp * headingError ));
  //printJustified((int)(Kd * deltaError * 1000 / deltaTime));
  //printJustified((int)cmd);
  display.clearDisplay();
  display.setCursor(5,5);
  display.print((int)(Kp * headingError ));
  display.print(" ");
  display.print((int)(Kd * deltaError * 1000 / deltaTime));
  display.print(" c:");
  display.print((int)cmd);
  display.display();
#endif
#if SERIALDEBUG==2
  Serial.print(deltaTime);
  Serial.print(" ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(headingError);
  Serial.print(" ");
  Serial.print(deltaError);
  Serial.print(" ");
  Serial.println(cmd);
#endif
  return (int)cmd; // casts cmd to an integer
}
/***********************************************************
   Buttons management
 ***********************************************************
*/
// This function will be called when the button1 was pressed 1 time
void b1Click() { // One degree to starboard
  bearing = map360(bearing + 1);
  beep(true);
}
void b1LongPress() { // During button pressed
  if (standby)  {
    // During Standby, if buttons 1 and 3 pressed, setup/calibration
    // else move tiller
    if (button3.isLongPressed())
      setupFonctions = true;
    else
      tillerCommand(-100);
  }
  else {
    beep(true);
    if (!launchTack && button2.isLongPressed()) { // Tack if two buttons pressed
      launchTack = true;
      bearing = map360(bearing + TACKANGLE);
    }
  }
}
void b1LongPressStop() { // Move to starboard
  if (!launchTack) bearing = map360(bearing + 20);
  if (standby) tillerStandby(true); // Stop the tiller at the end of press during standby
}
void b2Click() { // standby / Run
  beep(true);
  standby = !standby;
  tillerStandby(standby);
}
/* Dbl click replaced by simple click
 *  
 void b2DoubleClick() { // standby on
  if (!standby) beep(true);
  standby = true;
  tillerStandby(standby);
}
*/
void b2LongPressStop() { // reset steering or launch tack
  beep(true);
  if (launchTack)
    launchTack = false; // steering update already done
  else {
    display.clearDisplay();
    display.setCursor(5,5);
    display.print("Reset steering");
    display.display();
    bearing = heading;
    //    delay (100);
  }
}
void b3Click() { // One degree to badboard
  beep(true);
  bearing = map360(bearing - 1);
}
void b3LongPress() { // During button pressed
  if (standby)  {
    // During Standby, if buttons 1 and 3 pressed, calibration
    // else move tiller
    if (button1.isLongPressed())
      setupFonctions = true;
    else
      tillerCommand(+100);
  }
  else {
    beep(true);
    if (!launchTack && button2.isLongPressed()) { // Tack if two buttons pressed
      launchTack = true;
      bearing = map360(bearing - TACKANGLE);
    }
  }
}

void b3LongPressStop() { // Move to badboard
  if (!launchTack) bearing = map360(bearing - 20);
  if (standby) tillerStandby(true); // Stop the tiller at the end of press during standby
}

void buttonsInit() {
  static const int clickTicks = 300, pressTicks = 800;
  // link the button 1 functions.
  button1.setClickTicks(clickTicks);
  button1.setPressTicks(pressTicks);
  button1.attachClick(b1Click);
  // button2.attachDoubleClick(b1DoubleClick);
  //button1.attachLongPressStart(b1LongPressStart);
  button1.attachLongPressStop(b1LongPressStop);
  button1.attachDuringLongPress(b1LongPress);

  // link the button 2 functions.
  button2.setClickTicks(clickTicks);
  button2.setPressTicks(pressTicks);
  button2.attachClick(b2Click);
//  button2.attachDoubleClick(b2DoubleClick);
  //button2.attachLongPressStart(b2LongPressStart);
  button2.attachLongPressStop(b2LongPressStop);
  //button2.attachDuringLongPress(b2LongPress);

  // link the button 3 functions.
  button3.setClickTicks(clickTicks);
  button3.setPressTicks(pressTicks);
  button3.attachClick(b3Click);
  // button3.attachDoubleClick(b3DoubleClick);
  //button3.attachLongPressStart(b3LongPressStart);
  button3.attachLongPressStop(b3LongPressStop);
  button3.attachDuringLongPress(b3LongPress);
}

void buttonsTick() {
  beep(false);
  button1.tick();
  button2.tick();
  button3.tick();
}

/***********************************************************
   Initial setup
 ***********************************************************
*/
void setup() {

  Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
#if SERIALDEBUG
  Serial.begin(115200);
  Serial.println("Initializing...");
#endif
  pinMode(LED_PIN, OUTPUT);
  //pinMode(BACKLIGHTPIN, OUTPUT);

  // initialize display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  // display a line of text
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(5,5);
  display.print("Init "); display.display();

  calLibRead(0, &params);                           // pick up existing mag data (and other params) if there
  // NEXT
  Kp = params.Kp;
  Kd = params.Kd;

  deadband = params.deadband;
  display.print("*"); display.display();
  buttonsInit();
  display.print("*"); display.display();
  tillerInit();
  display.print("*"); display.display();

  // for LSM303D compass
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-3613, -3417, -3472}; // use values from calibration
  compass.m_max = (LSM303::vector<int16_t>){+3592, +3243, +3907}; // use values from calibration
  compass.read();
  bearing = compass.heading();
  display.print("*"); display.display();
}

/***********************************************************
   Main loop
 ***********************************************************
*/

  float headingSum = 0;
  int headingCnt = 0;
void loop() {
  /*
     Main loop
   	read keypad/buttons
   	read heading
   	compute steering (PID controller)
   	steer
  */
  int ctrl=0;
  
  buttonsTick();
  if (setupFonctions) {
    tillerCommand(0); // Security...
    setupMenu();
    setupFonctions = false;
  }
  // Sum for heading average. LSM303D
  compass.read();
  headingSum += compass.heading();
  headingCnt++;
  #if SERIALDEBUG
    //Serial.print("headingSum: ");
    //Serial.println(headingSum);
  #endif
 
  unsigned long now = millis();

 // Periodic calculation of the steering control (with heading average)
  if ((now - prevCommand) >= COMMAND_INTERVAL) {
    prevCommand = now;
    heading = headingSum/headingCnt;
#if DEBUG == 1
  display.clearDisplay();
  display.setCursor(5,5);
  display.print("hc ");
  display.println(headingCnt);
  display.display();
#endif
	  headingSum = 0;
	  headingCnt = 0;
    ctrl = computeCmd();
#if DEBUG == 1
  display.print("c ");
  display.println(ctrl);
  display.display();
#endif
  }

  // Control running tiller actuator
  if (!standby)
      tillerCommand(ctrl);

  // Periodic display refresh
  if ((now - prevDisplay) >= DISPLAY_INTERVAL) {
    prevDisplay = now;

    display.clearDisplay();
    display.setCursor(5,5);
    if (standby)
      display.println("Sby ");
    else
      display.println("Run ");

    display.print("b ");
    display.print(round(bearing));
    display.print(" h ");
    display.print(round(heading));
    display.print(" e ");
    display.println(round(headingError));
    display.print("Kp ");
    display.print(Kp);
    display.print(" Kd ");
    display.print(Kd);
    display.display();

    //printJustified(tillerCurrent);
  }

}
