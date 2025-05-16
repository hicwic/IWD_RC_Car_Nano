#include <Arduino.h>
#include <Servo.h>

// Debug flag
#define DEBUG false

#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, y) Serial.print(x, y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, y)
#endif

// === CONFIGURATION ===

// Number of RC input channels
#define NUM_CHANNELS 4
const uint8_t chPins[NUM_CHANNELS] = {2, 3, 4, 5}; // Direction, Throttle, Mix, Freewheel

// ESC output pins
const int escLeftPin = 9;
const int escRightPin = 10;

Servo escLeft;
Servo escRight;

// PWM signal range in microseconds
const int pwmMin = 1040;
const int pwmMid = 1460;
const int pwmMax = 1960;
const int pwmArm = 1460; // Arming signal for BLHeli_S ESCs

// Thresholds and limits
const int deadzone = 10;           // Joystick deadband
const int maxMix = 50;             // Max differential mix (%)
const int minThrottleFwd = 10;     // Minimum throttle in forward to avoid zero PWM

// Reverse mode state flags
bool reverseMode = false;
bool readyForReverse = false;
bool invertSteeringMix = false;     // Invert steering if necessary for correct direction

// Reverse safety timing
unsigned long neutralStartTime = 0;
const unsigned long neutralDelay = 500; // Time (ms) in neutral before allowing reverse

// Motion velocity and decay parameters
float baseVelocity = 0.0;               // Virtual vehicle speed (-100 to 100)
float targetVelocity = 0.0;    
float velocityDecayRate = 20.0;         // % per second, adjustable via channel 4

// Ramp & Boost
bool ramping = false;
float rampTargetVelocity = 0.0;
float rampStep = 0.0;
uint8_t rampStepCount = 0;

const uint8_t RAMP_STEPS = 6;
const uint8_t RAMP_INTERVAL_MS = 15;
const float START_BOOST_PWM = 120;
unsigned long lastRampTime = 0;

// Loop timing
const unsigned long loopInterval = 10;  // Loop update interval in ms (100Hz)
unsigned long lastLoopTime = 0;

// LED status blinking
unsigned long lastBlinkTime = 0;
bool ledState = false;

// PWM reading via interrupts
volatile unsigned long risingTime[NUM_CHANNELS] = {0};      // Capture rising edge time
volatile unsigned int pulseWidth[NUM_CHANNELS] = {pwmMid, pwmMid, pwmMid, pwmMid}; // Store width
volatile uint8_t pinState = 0;  // Current state of input pins

// === UTILITY FUNCTIONS ===

// Convert PWM (1000-2000µs) to percent (-100 to 100)
int pwmToPercent(int pwm) {
  return map(pwm, pwmMin, pwmMax, -100, 100);
}

// Convert percent (-100 to 100) to PWM signal (1000-2000µs)
int percentToPWM(float percent) {
  percent = constrain(percent, -100, 100);
  if (percent > 0) return map(percent, 0, 100, pwmMid, pwmMax);
  if (percent < 0) return map(-percent, 0, 100, pwmMid, pwmMin);
  return pwmMid;
}

// Apply gradual decay of a value toward zero at a rate (percent/second)
float decayTowardsZero(float val, float ratePerSecond, float deltaTime) {
  float step = ratePerSecond * deltaTime;
  if (val > step) return val - step;
  if (val < -step) return val + step;
  return 0;
}

// === INTERRUPT HANDLER FOR RC INPUTS ===

// Handles all pin change interrupts on PORTD (pins 2 to 7)
ISR(PCINT2_vect) {
  uint8_t currentPins = PIND;
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    uint8_t bitMask = (1 << chPins[i]);
    bool previous = pinState & bitMask;
    bool current = currentPins & bitMask;

    // Rising or falling edge detected
    if (current != previous) {
      if (current) {
        // Rising edge - start timing
        risingTime[i] = micros();
      } else {
        // Falling edge - compute pulse width
        unsigned long width = micros() - risingTime[i];
        if (width >= 500 && width <= 2500) pulseWidth[i] = width;
        else pulseWidth[i] = pwmMid; // Fallback to neutral
      }
    }
  }
  pinState = currentPins;
}

// === SETUP FUNCTION ===

void setup() {
  Serial.begin(115200);

  // Initialize built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Set input pins for RC channels
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) pinMode(chPins[i], INPUT);

  // Attach ESCs to output pins with PWM limits
  escLeft.attach(escLeftPin, pwmMin, pwmMax);
  escRight.attach(escRightPin, pwmMin, pwmMax);

  // Configure pin change interrupt on pins 2 to 5
  pinState = PIND;
  PCICR |= (1 << PCIE2); // Enable PCINT2 interrupt
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);

  // Send arming signal to ESCs
  delay(2000);
  escLeft.writeMicroseconds(pwmArm);
  escRight.writeMicroseconds(pwmArm);
  delay(2000);

  DEBUG_PRINTLN("ESCs armed.");
  lastLoopTime = millis();
}

// === MAIN CONTROL LOOP ===

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime >= loopInterval) {
    float deltaTime = (now - lastLoopTime) / 1000.0;

    // Read all channel pulse widths atomically
    noInterrupts();
    int dirPWM = pulseWidth[0];
    int throttlePWM = pulseWidth[1];
    int mixPWM = pulseWidth[2];
    int decayPWM = pulseWidth[3];
    interrupts();

    // Convert to percentage values
    int dirPercent = pwmToPercent(dirPWM);
    int throttlePercent = pwmToPercent(throttlePWM);
    int mixPercent = map(mixPWM, pwmMin, pwmMax, 0, maxMix);
    velocityDecayRate = map(decayPWM, pwmMin, pwmMax, 20, 100); // Decay from channel 4

    const int mixSign = invertSteeringMix ? -1 : 1;

    // === MOTION STATE MANAGEMENT ===

    if (throttlePercent > deadzone) {
      // Forward motion
      reverseMode = false;
      readyForReverse = false;
      neutralStartTime = 0;

      targetVelocity = throttlePercent;


    } else if (throttlePercent >= -deadzone && throttlePercent <= deadzone) {
      // Neutral zone
      reverseMode = false;
      ramping = false;
      throttlePercent = 0;
      targetVelocity = 0;

      // Check if neutral long enough to allow reverse
      bool isNeutral = abs(baseVelocity) < 1.0;
      if (isNeutral) {
        if (neutralStartTime == 0) neutralStartTime = now;
        else if (now - neutralStartTime >= neutralDelay) readyForReverse = true;
      } else {
        neutralStartTime = 0;
        readyForReverse = false;
      }

    } else {
      // Braking
      if (!readyForReverse) {
        reverseMode = false;
        ramping = false;
        baseVelocity = 0;
        targetVelocity = 0;
      } 
      // Reverse
      else {
        reverseMode = true;
        targetVelocity = throttlePercent;
      }
    }


    // === COASTING LOGIC ===

    if (abs(baseVelocity) > abs(targetVelocity) && !ramping) {
      int direction = baseVelocity/abs(baseVelocity);
      baseVelocity = direction * max(decayTowardsZero(abs(baseVelocity), velocityDecayRate, deltaTime), abs(targetVelocity));    
    }


    // === RAMPING LOGIC ===

    if (baseVelocity == 0 && abs(targetVelocity) > 0 && !ramping) {
      int direction = targetVelocity/abs(targetVelocity);

      // Start from stop: apply boost + prepare ramp
      escLeft.writeMicroseconds(pwmMid + START_BOOST_PWM * direction);
      escRight.writeMicroseconds(pwmMid + START_BOOST_PWM * direction);
      delayMicroseconds(5000); // short boost (~5 ms)

      ramping = true;
      rampTargetVelocity = targetVelocity;
      rampStepCount = 0;
      rampStep = rampTargetVelocity / RAMP_STEPS;
      baseVelocity = 0;
      lastRampTime = now;
    }

    if (ramping) {
      if (abs(targetVelocity) < abs(rampTargetVelocity)) {
        ramping = false; //cancel ramping if targetVelocity suddently fall bellow ramping target velocity
       } else if (now - lastRampTime >= RAMP_INTERVAL_MS) {
        baseVelocity += rampStep;
        rampStepCount++;
        lastRampTime = now;
        if (rampStepCount >= RAMP_STEPS) {
          baseVelocity = rampTargetVelocity;
          ramping = false;
        }
      }
    }

    // === DIFFERENTIAL LOGIC ===

    float diff = dirPercent * mixPercent / 100.0;

    float leftVelocity  = baseVelocity - mixSign * diff;
    float rightVelocity = baseVelocity + mixSign * diff;

    // Enforce throttle floor for forward motion
    if (baseVelocity > 0) {
      leftVelocity  = constrain(leftVelocity,  minThrottleFwd, 100);
      rightVelocity = constrain(rightVelocity, minThrottleFwd, 100);
    }
    else if (baseVelocity == 0) {
      leftVelocity = 0;
      rightVelocity = 0;
    }
    else {
      leftVelocity  = constrain(leftVelocity,  -100, 0);
      rightVelocity = constrain(rightVelocity, -100, 0);
    }

    // Convert to PWM and send to ESCs
    int pwmLeft  = percentToPWM(leftVelocity);
    int pwmRight = percentToPWM(rightVelocity);

    escLeft.writeMicroseconds(pwmLeft);
    escRight.writeMicroseconds(pwmRight);

    // === DEBUG OUTPUT ===
    DEBUG_PRINT("Throttle %: "); DEBUG_PRINT(throttlePercent);
    DEBUG_PRINT(" | Dir %: "); DEBUG_PRINT(dirPercent);
    DEBUG_PRINT(" | Mix %: "); DEBUG_PRINT(mixPercent);
    DEBUG_PRINT(" | Step %/s: "); DEBUG_PRINT(velocityDecayRate);
    DEBUG_PRINT(" | BaseVel: "); DEBUG_PRINTF(baseVelocity, 1);
    DEBUG_PRINT(" | LVel: "); DEBUG_PRINTF(leftVelocity, 1);
    DEBUG_PRINT(" | RVel: "); DEBUG_PRINTF(rightVelocity, 1);
    DEBUG_PRINT(" | Reverse: "); DEBUG_PRINT(reverseMode ? "Yes" : "No");
    DEBUG_PRINT(" | ReadyRev: "); DEBUG_PRINT(readyForReverse ? "Yes" : "No");
    DEBUG_PRINT(" | PWM L: "); DEBUG_PRINT(pwmLeft);
    DEBUG_PRINT(" | PWM R: "); DEBUG_PRINT(pwmRight);  
    DEBUG_PRINT(" | PWM THR: "); DEBUG_PRINT(throttlePWM);  
    DEBUG_PRINT(" | DeltaTime: "); DEBUG_PRINTLN(deltaTime);    

    // === LED BLINKING FEEDBACK ===

    if (baseVelocity != 0.0) {
      // Blink LED at rate proportional to speed (max 10Hz)
      float frequencyHz = abs(baseVelocity) / 10.0;  // 0 to 10 Hz
      float period = 1000.0 / frequencyHz;           // ms period
      float halfPeriod = period / 2.0;

      if (millis() - lastBlinkTime >= halfPeriod) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        lastBlinkTime = millis();
      }
    } else {
      // LED off when not moving
      ledState = false;
      digitalWrite(LED_BUILTIN, LOW);
    }

    // Update loop timer
    lastLoopTime = now;
  }  
}
