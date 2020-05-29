#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>  // https://github.com/PaulStoffregen/Encoder


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
// on the uno, only 2 and 3 have interrupt capability
// and d2 is used by the infrared led
Encoder myEnc(3, 4);
long oldPosition = -999;

// IR sensor on D2
const byte interruptPin = 2;
int numBalls = 0;
int lastNumBalls = 0;


// rotary button on D5
const byte rotaryButton = 5;


// sda=A4, slc=A5
// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16,2);


/*
 * PWM 3, 5, 6, 9, 10, 11
 * 
 * Pins 5 and 6 are paired on timer0, with base frequency of 62500Hz
 * Pins 9 and 10 are paired on timer1, with base frequency of 31250Hz
 * Pins 3 and 11 are paired on timer2, with base frequency of 31250Hz
 * 
 * Pins 5 and 6 have prescaler values of 1, 8, 64, 256, and 1024
 * Pins 9 and 10 have prescaler values of 1, 8, 64, 256, and 1024
 * Pins 3 and 11 have prescaler values of 1, 8, 32, 64, 128, 256, and 1024
 * 
 * For pins 6 and 5 (OC0A and OC0B):
 * If TCCR0B = xxxxx001, frequency is 64kHz
 * If TCCR0B = xxxxx010, frequency is 8 kHz
 * If TCCR0B = xxxxx011, frequency is 1kHz (this is the default from the Diecimila bootloader)
 * If TCCR0B = xxxxx100, frequency is 250Hz
 * If TCCR0B = xxxxx101, frequency is 62.5 Hz
 * 
 * For pins 9, 10, 11 and 3 (OC1A, OC1B, OC2A, OC2B):
 * If TCCRnB = xxxxx001, frequency is 32kHz
 * If TCCRnB = xxxxx010, frequency is 4 kHz
 * If TCCRnB = xxxxx011, frequency is 500Hz (this is the default from the Diecimila bootloader)
 * If TCCRnB = xxxxx100, frequency is 125Hz
 * If TCCRnB = xxxxx101, frequency is 31.25 Hz
 */
// flywheel motors on D6
// feed motor on D9
const byte flywheelMotor = 6;
const byte feedMotor = 9;

// this is the amount of time we wait for the flywheel motors to spin up
// before turning on the feed motor  (msec)
// it may vary depending on the speed...but then again, maybe not
const long flywheelMotorSpinup = 1000;

// flywheel button on D8
// feed button on D7
const byte flywheelButton = 8;
const byte feedButton = 7;

char msg[12];


// debug
//int speed = 0;


enum shotModes { SINGLE, BURST, FULL, MANUAL };
const char *shotModeNames[] = {"single", "burst", "full", "manual"};
int shotMode = SINGLE;

enum speedModes { LOW_SPEED, MEDIUM_SPEED, HIGH_SPEED };
const char *speedModeNames[] = {"low", "med", "high"};
const byte speedPwm[] = {40, 70, 100};    // may need to fine tune this
int speedMode = LOW_SPEED;

enum feedModes { LOW_FEED, MEDIUM_FEED, HIGH_FEED };
const char *feedModeNames[] = {"low", "med", "high"};
const byte feedPwm[] = {40, 70, 100};    // may need to fine tune this
int feedMode = LOW_FEED;

enum adjustValues { SHOT, SPEED, FEED, BALL };
const char *adjustNames[] = {"shot", "speed", "feed", "ball"};
int adjust = SHOT;
int lastAdjust = adjust;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
const long debounceDelay = 50;    // the debounce time

const int defaultButtonState = HIGH;
int buttonState = defaultButtonState;
int lastButtonState = defaultButtonState;

bool isJammed = false;

// allow the gun to fire by turning the encoder up in ball mode
// this should only be used for debug...seems dangereous
bool rotaryEncoderFiring = true;

bool triggersIdle = false;


void setup() {
  // set up the pwm outputs
  pinMode(flywheelMotor, OUTPUT);
  pinMode(feedMotor, OUTPUT);
  analogWrite(flywheelMotor, 0);
  analogWrite(feedMotor, 0);

  // set the pwm frequency
  // to reduce motor whine...needed?
  // timer 0, pin D6, 8 kHz
//  TCCR0A = TCCR0A & 0b11111000 | 0b010;
  // timer 1, pin D9, 4 kHz
//  TCCR1A = TCCR1A & 0b11111000 | 0b010;
  
  Serial.begin(115200);
  Serial.println("nerf gun");
  Serial.println("compiled:");
  Serial.print( __DATE__);
  Serial.print(",");
  Serial.println( __TIME__);

  if (rotaryEncoderFiring)
    Serial.println("\nWARNING: rotaryEncoderFiring is enabled\n");
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ball, FALLING);

  pinMode(rotaryButton, INPUT_PULLUP);

  pinMode(flywheelButton, INPUT_PULLUP);
  pinMode(feedButton, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  drawDisplay();
}

void drawDisplay() {
  lcd.clear();
  drawBallCount();
  drawShotMode();
  drawSpeedMode();
  drawFeedMode();
  drawMode(adjust, '>');
}

void drawBallCount() {
  // show ball count
  sprintf(msg, "%3d", numBalls);
  lcd.setCursor(1,0);
  lcd.print(msg);
}

void drawShotMode() {
  // show shot mode
  sprintf(msg, "%6s", shotModeNames[shotMode]);
  lcd.setCursor(9,0);
  lcd.print(msg);
}

void drawSpeedMode() {
  // show speed mode
  sprintf(msg, "S:%4s", speedModeNames[speedMode]);
  lcd.setCursor(1,1);
  lcd.print(msg);
}

void drawFeedMode() {
  // show feed mode
  sprintf(msg, "F:%4s", feedModeNames[feedMode]);
  lcd.setCursor(9,1);
  lcd.print(msg);
}

void drawMode(int mode, char ch) {
  int row = (mode == BALL || mode == SHOT) ? 0 : 1; 
  int col = (mode == BALL || mode == SPEED) ? 0 : 8; 
  lcd.setCursor(col,row);
  lcd.print(ch);
}

void loop() {
//  loopDebug();

  if (!isJammed) {
    handleEncoderMovement();

    // test code
//    long start = 0;
//    if (numBalls == 0) {
//      numBalls = 3;
//      start = millis();
//    }
    drawBall();
//    if (start != 0) {
//      long delta = millis() - start;
//      Serial.println(delta);   // took 6 msec to update the ball count
//    }
    
    handleTrigger();
  }
  handleEncoderButton();
}


bool isFlywheelOn = false;
bool isFeedOn = false;


void handleTrigger() {
  if (shotMode == MANUAL) {
    // in manual mode the trigger control the motors directly
    int fwb = digitalRead(flywheelButton);
    if (fwb == LOW) {
      // flywheel trigger pressed
      if (!isFlywheelOn) {
        // turn flywheel motor on
        isFlywheelOn = true;
        analogWrite(flywheelMotor, speedPwm[speedMode]);
        Serial.println("flywheel on");
      }
    }
    else {
      // flywheel trigger released
      if (isFlywheelOn) {
        // turn flywheel motor off
        isFlywheelOn = false;
        analogWrite(flywheelMotor, 0);
        Serial.println("flywheel off");
      }
    }

    int fb = digitalRead(feedButton);
    if (fb == LOW) {
      // feed trigger pressed
      if (!isFeedOn) {
        // turn feed motor on
        isFeedOn = true;
        analogWrite(feedMotor, feedPwm[feedMode]);
        Serial.println("feed on");
      }
    }
    else {
      // feed trigger released
      if (isFeedOn) {
        // turn feed motor off
        isFeedOn = false;
        analogWrite(feedMotor, 0);
        Serial.println("feed off");
      }
    }
  }
  else {
    // wait for both buttons to be idle before allowing
    // them to fire again.  (unless full auto)
    int fwb = digitalRead(flywheelButton);
    int fb = digitalRead(feedButton);
    if (fwb == HIGH && fb == HIGH) {
      triggersIdle = true;
      return;
    }
  
    if (!triggersIdle)
      return;
      
    if (fwb == LOW || fb == LOW) {
      triggersIdle = false;
      fire(); 
    }
  }
}

void fire() {
  Serial.println("fire");
  
  // take note of the current shot count
  int startBallCount = numBalls;
  
  // spin up the flywheel motors
  analogWrite(flywheelMotor, speedPwm[speedMode]);

  // wait for them to get up to speed
  delay(flywheelMotorSpinup);

  // spin up the feed motor
  analogWrite(feedMotor, feedPwm[feedMode]);

  // stop the motors if we've reached our shot count (single or burst)
  // or the shot counter is not incrementing
  //   i.e. the ball counter hasn't incremented in the last second
  // or the button is released (in full auto mode)
  int jamCount = 0;
  if (shotMode == FULL) {
    while (jamCount < 10 && 
           (digitalRead(flywheelButton) == LOW || digitalRead(feedButton) == LOW)) {
      startBallCount = numBalls;
      updateBallCountAndDelay(100);
      if (numBalls == startBallCount)
        ++jamCount;
      else
        jamCount = 0;
    }
  }
  else {
    // single or burst
    int target = startBallCount + (shotMode == SINGLE) ? 1 : 3;
    while (jamCount < 10 && numBalls < target) {
      startBallCount = numBalls;
      updateBallCountAndDelay(100);
      if (numBalls == startBallCount)
        ++jamCount;
      else
        jamCount = 0;
    }
  }

  // turn everything off
  analogWrite(feedMotor, 0);
  analogWrite(flywheelMotor, 0);

  Serial.println("stopped");

  // if there was a jam, tell the user
  if (jamCount >= 10) {
    jammed();
  }
}

void updateBallCountAndDelay(long totalDelay) {
  // get the current millisec then call the update call, then delay
  // for totalDelay - time spent in update call.
  long start = millis();
  drawBall();    // this takes around 6 msec to run (if there is a ball update)
                 // maybe longer for ball counts of more than a single digit
  long delta = millis() - start;
  delay(totalDelay - delta);
}

void jammed() {
  Serial.println("jammed");
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("J A M M E D");
  isJammed = true;
}

void drawBall() {
  // check for a ball
  if (numBalls != lastNumBalls) {
    lastNumBalls = numBalls;
    drawBallCount();
//    Serial.println(numBalls);
  }
}

void handleEncoderButton() {
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:
  int reading = digitalRead(rotaryButton);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      if (reading != defaultButtonState)
        doButton();
    }
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
}

void doButton() {
  // if we're jammed, pressing the button resets it
  if (isJammed) {
    isJammed = false;
    Serial.println("jam reset");
    drawDisplay();
    return;
  }
  
  if (++adjust > BALL)
    adjust = SHOT;
  Serial.println(adjustNames[adjust]);

  // update the display
  drawMode(lastAdjust, ' ');
  lastAdjust = adjust;
  drawMode(adjust, '>');
}

void handleEncoderMovement() {
  int dir = readEncoder();
  if (dir != 0) {
    if (adjust == SHOT) {
      if (dir > 0) {
        if (++shotMode > MANUAL)
          shotMode = SINGLE;
      }
      else {
        if (--shotMode < SINGLE)
          shotMode = MANUAL;
      }
      drawShotMode();
    }
    else if (adjust == SPEED) {
      if (dir > 0) {
        if (++speedMode > HIGH_SPEED)
          speedMode = LOW_SPEED;
      }
      else {
        if (--speedMode < LOW_SPEED)
          speedMode = HIGH_SPEED;
      }
      drawSpeedMode();
    }
    else if (adjust == FEED) {
      if (dir > 0) {
        if (++feedMode > HIGH_FEED)
          feedMode = LOW_FEED;
      }
      else {
        if (--feedMode < LOW_FEED)
          feedMode = HIGH_FEED;
      }
      drawFeedMode();
    }
    else {
      // ball
      if (rotaryEncoderFiring && dir > 0) {
        // allow firing via the rotary encoder...seems dangereous
        fire();
      }
      else {
        // just reset...it'll get redrawn automatically
        Serial.println("reset ball count");
        numBalls = 0;
      }
    }
  }
}

int readEncoder() {
  // read rotary encoder
  int dir = 0;
  long newPosition = myEnc.read();
//  Serial.println(newPosition);
  if (newPosition != oldPosition) {
    if (oldPosition != -999) {
      if (newPosition < oldPosition-3) {
        oldPosition = newPosition;
        dir = -1;
      }
      else if (newPosition > oldPosition+3) {
        oldPosition = newPosition;
        dir = 1;
      }
    }
    else
      oldPosition = newPosition;
  }
  return dir;
}

//void lookDebug() {
//  // read rotary encoder
//  long newPosition = myEnc.read();
//  if (newPosition != oldPosition) {
//    if (oldPosition == -999)
//      // ignore
//      ;
//    else if (newPosition < oldPosition)
//      speed = max(0, speed-1);
//    else
//      speed = min(255, speed+1);
//    analogWrite(flywheelMotor, speed);
//    oldPosition = newPosition;
//    int speedPercent = round((float)speed * 100 / 255);
//    sprintf(msg, "%3d%%", speedPercent);
//    Serial.println(msg);
//    lcd.setCursor(7,0);
//    lcd.print(msg);
//  }
//
//  // check for a ball
//  if (numBalls != lastNumBalls) {
//    lastNumBalls = numBalls;
//    sprintf(msg, "%3d", numBalls);
//    Serial.println(msg);
//    lcd.setCursor(7,1);
//    lcd.print(msg);
//  }
//
//  // check for rotary encoder button
//  if (digitalRead(rotaryButton) == 0) {
//    numBalls = 0;
//  }
//}

void ball() {
  ++numBalls;
}


/*
 * todo:
 * save the mode values in eeprom when they change, and read at startup
 * add a power switch
 * add electronic motor braking
 * can ball speed be measured with the isr (time btween rising and falling edges?)
 * 
 * during tests, the ball counter would sometimes measure high.   why?
 */
