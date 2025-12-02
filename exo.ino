/*
 * Single-channel EMG → Servo controller with:
 * - Deterministic-ish sampling via micros()
 * - Simple digital filtering
 * - Calibration and normalization
 * - Text-based menu UI on Serial Monitor
 * - 3-button navigation (UP / DOWN / SELECT)
 *
 * Target: Arduino Nano (ATmega328P)
 */

#include <Servo.h>
#include <EEPROM.h>

// ---------------------- Hardware config ----------------------

#define EMG_PIN          A0
#define SERVO_PIN        3

#define BTN_UP_PIN       12
#define BTN_DOWN_PIN     10
#define BTN_SELECT_PIN   11

// ---------------------- Timing config ------------------------

const unsigned long SAMPLE_PERIOD_US   = 1000;   // ~1 kHz EMG sampling
const unsigned long SERVO_UPDATE_MS    = 20;     // ~50 Hz servo update
const unsigned long UI_UPDATE_MS       = 200;    // 5 Hz UI update

// ---------------------- EMG filtering/processing ----------------------

volatile uint16_t emgRaw           = 0;   // last raw sample
float emgFiltered                  = 0.0; // filtered signal
float emgNorm                      = 0.0; // normalized 0..1

// Exponential moving average (EMA) coefficient.
// Smaller alpha = more smoothing, more latency.
const float EMA_ALPHA               = 0.10f;

// For spike rejection (optional simple scheme)
const uint16_t SPIKE_THRESHOLD      = 150; // adjust based on noise floor

// For "flat signal" detection (optional, basic)
const unsigned long FLAT_CHECK_MS   = 2000;
const uint16_t FLAT_DELTA_THRESHOLD = 5;

// ---------------------- Calibration data ----------------------

struct CalibrationData {
  uint16_t baseline;  // EMG at rest
  uint16_t max;       // EMG at max voluntary contraction
  uint8_t  magic;
  uint8_t  checksum;
};

const uint8_t CAL_MAGIC        = 0x42;
const int     CAL_EEPROM_ADDR  = 0;   // starting address

CalibrationData cal;
bool calibrationValid = false;

// ---------------------- Servo control ----------------------

Servo servo;

float servoAngleCurrent = 0.0f;
float servoAngleTarget  = 0.0f;

const float SERVO_MIN_ANGLE  = 0.0f;
const float SERVO_MAX_ANGLE  = 120.0f; // adjust to your mechanics
const float SERVO_MAX_STEP   = 5.0f;   // max degrees per update step

// ---------------------- Buttons (debounced) ----------------------

struct Button {
  uint8_t pin;
  bool    stableState;      // debounced state (true = pressed)
  bool    lastStableState;  // previous stable state
  bool    pressedEvent;     // edge: pressed this cycle
  unsigned long lastChangeTime;
};

const unsigned long DEBOUNCE_MS = 30;

Button btnUp     = { BTN_UP_PIN,     false, false, false, 0 };
Button btnDown   = { BTN_DOWN_PIN,   false, false, false, 0 };
Button btnSelect = { BTN_SELECT_PIN, false, false, false, 0 };

// ---------------------- System modes & menu ----------------------

enum SystemMode {
  MODE_BOOT,
  MODE_IDLE,
  MODE_CAL_BASELINE,
  MODE_CAL_MAX,
  MODE_ACTIVE,
  MODE_ERROR
};

SystemMode mode = MODE_BOOT;

String errorMessage;

// Calibration accumulation
bool     calCollecting       = false;
uint32_t calAccum            = 0;
uint16_t calSamplesCollected = 0;
const uint16_t CAL_SAMPLES   = 1000; // number of samples for each phase (~1 sec at 1kHz)

// ---------------------- Timing variables ----------------------

unsigned long nextSampleTimeUs  = 0;
unsigned long lastServoUpdateMs = 0;
unsigned long lastUiUpdateMs    = 0;

// For flat signal detection
unsigned long lastFlatCheckMs   = 0;
uint16_t lastFlatCheckValue     = 0;
bool emgFlatFault               = false;

// ---------------------- Helper: checksum ----------------------

uint8_t calcChecksum(const CalibrationData &c) {
  uint8_t sum = 0;
  sum ^= (c.baseline & 0xFF);
  sum ^= (c.baseline >> 8);
  sum ^= (c.max & 0xFF);
  sum ^= (c.max >> 8);
  sum ^= c.magic;
  return sum;
}

// ---------------------- EEPROM handling ----------------------

void loadCalibration() {
  EEPROM.get(CAL_EEPROM_ADDR, cal);

  if (cal.magic != CAL_MAGIC) {
    calibrationValid = false;
    return;
  }

  uint8_t cs = calcChecksum(cal);
  if (cs != cal.checksum) {
    calibrationValid = false;
    return;
  }

  if (cal.max <= cal.baseline) {
    calibrationValid = false;
    return;
  }

  calibrationValid = true;
}

void saveCalibration() {
  cal.magic    = CAL_MAGIC;
  cal.checksum = calcChecksum(cal);
  EEPROM.put(CAL_EEPROM_ADDR, cal);
  calibrationValid = true;
}

// ---------------------- Button handling ----------------------

void updateButton(Button &b) {
  bool raw = (digitalRead(b.pin) == LOW); // using INPUT_PULLUP: LOW = pressed
  unsigned long now = millis();

  if (raw != b.stableState) {
    // potential change
    if ((now - b.lastChangeTime) >= DEBOUNCE_MS) {
      // accept new state
      b.lastStableState = b.stableState;
      b.stableState = raw;
      b.lastChangeTime = now;

      // detect rising edge (not pressed -> pressed)
      if (!b.lastStableState && b.stableState) {
        b.pressedEvent = true;
      }
    }
  } else {
    // no change
    b.pressedEvent = false;
  }
}

void updateButtons() {
  updateButton(btnUp);
  updateButton(btnDown);
  updateButton(btnSelect);
}

// ---------------------- EMG sampling & processing ----------------------

void sampleEMG() {
  uint16_t raw = analogRead(EMG_PIN);
  emgRaw = raw;

  // Simple spike rejection
  float prevFiltered = emgFiltered;
  if (fabs((float)raw - prevFiltered) > SPIKE_THRESHOLD) {
    if (raw > prevFiltered) {
      raw = (uint16_t)(prevFiltered + SPIKE_THRESHOLD);
    } else {
      raw = (uint16_t)(prevFiltered - SPIKE_THRESHOLD);
    }
  }

  // Exponential moving average
  emgFiltered = emgFiltered + EMA_ALPHA * ((float)raw - emgFiltered);

  // Flat signal detection
  unsigned long nowMs = millis();
  if (nowMs - lastFlatCheckMs >= FLAT_CHECK_MS) {
    uint16_t current = (uint16_t)emgFiltered;
    if (abs((int)current - (int)lastFlatCheckValue) <= FLAT_DELTA_THRESHOLD) {
      emgFlatFault = true;
    } else {
      emgFlatFault = false;
    }
    lastFlatCheckValue = current;
    lastFlatCheckMs = nowMs;
  }

  // Normalization, only if calibration is valid and max > baseline
  if (calibrationValid && cal.max > cal.baseline) {
    float num = emgFiltered - (float)cal.baseline;
    float den = (float)(cal.max - cal.baseline);
    float n = num / den;
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    emgNorm = n;
  } else {
    emgNorm = 0.0f;
  }

  // Handle calibration accumulation if active
  if (calCollecting) {
    calAccum += raw;
    calSamplesCollected++;

    if (calSamplesCollected >= CAL_SAMPLES) {
      calCollecting = false;
      // Next step handled in state machine
    }
  }
}

// ---------------------- Servo control ----------------------

void updateServo() {
  unsigned long nowMs = millis();
  if (nowMs - lastServoUpdateMs < SERVO_UPDATE_MS) return;
  lastServoUpdateMs = nowMs;

  // If there is any critical fault, move to safe position
  if (emgFlatFault || !calibrationValid || mode == MODE_ERROR || mode == MODE_BOOT) {
    servoAngleTarget = SERVO_MIN_ANGLE;
  } else if (mode == MODE_ACTIVE) {
    // Map normalized EMG to servo range
    servoAngleTarget = SERVO_MIN_ANGLE + (1.0f - emgNorm) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
  } else {
    // Idle or calibration modes: keep at minimum angle
    servoAngleTarget = SERVO_MIN_ANGLE;
  }

  // Rate limit
  float delta = servoAngleTarget - servoAngleCurrent;
  if (delta > SERVO_MAX_STEP) delta = SERVO_MAX_STEP;
  if (delta < -SERVO_MAX_STEP) delta = -SERVO_MAX_STEP;

  servoAngleCurrent += delta;

  // Saturation
  if (servoAngleCurrent < SERVO_MIN_ANGLE) servoAngleCurrent = SERVO_MIN_ANGLE;
  if (servoAngleCurrent > SERVO_MAX_ANGLE) servoAngleCurrent = SERVO_MAX_ANGLE;

  servo.write((int)servoAngleCurrent);
}

// ---------------------- Text UI helpers (Serial) ----------------------

void printDivider() {
  Serial.println(F("--------------------------------------------------"));
}

void showBootScreen() {
  printDivider();
  Serial.println(F("MODE: BOOT"));
  Serial.println(F("Initializing EMG → Servo controller..."));
}

void showErrorScreen() {
  printDivider();
  Serial.println(F("MODE: ERROR"));
  Serial.print(F("Error: "));
  Serial.println(errorMessage);
  Serial.println(F("Press SELECT to return to IDLE."));
}

void showIdleScreen() {
  printDivider();
  Serial.println(F("MODE: IDLE"));
  Serial.print(F("Calibration OK: "));
  Serial.println(calibrationValid ? F("YES") : F("NO"));

  Serial.print(F("EMG filtered: "));
  Serial.println((int)emgFiltered);

  Serial.print(F("Servo angle: "));
  Serial.println((int)servoAngleCurrent);

  Serial.println();
  Serial.println(F("Controls:"));
  Serial.println(F("  SELECT : Start calibration"));
  Serial.println(F("  UP     : Enter ACTIVE mode (if calibrated)"));
}

void showActiveScreen() {
  printDivider();
  Serial.println(F("MODE: ACTIVE"));

  Serial.print(F("EMG filtered: "));
  Serial.println((int)emgFiltered);

  Serial.print(F("EMG normalized: "));
  Serial.println(emgNorm, 2);

  Serial.print(F("Servo angle: "));
  Serial.println((int)servoAngleCurrent);

  Serial.println();
  Serial.println(F("Controls:"));
  Serial.println(F("  SELECT : Return to IDLE"));
}

void showCalBaselineScreen(bool collecting, uint16_t samples) {
  printDivider();
  Serial.println(F("MODE: CALIBRATION (BASELINE)"));
  Serial.println(F("Instruction: Keep hand RELAXED."));

  if (!collecting) {
    Serial.println(F("Press SELECT to start baseline capture."));
  } else {
    Serial.print(F("Capturing baseline samples: "));
    Serial.print(samples);
    Serial.print(F(" / "));
    Serial.println(CAL_SAMPLES);
  }
}

void showCalMaxScreen(bool collecting, uint16_t samples) {
  printDivider();
  Serial.println(F("MODE: CALIBRATION (MAX)"));
  Serial.println(F("Instruction: SQUEEZE as hard as possible."));

  if (!collecting) {
    Serial.println(F("Press SELECT to start max capture."));
  } else {
    Serial.print(F("Capturing max samples: "));
    Serial.print(samples);
    Serial.print(F(" / "));
    Serial.println(CAL_SAMPLES);
  }
}

// ---------------------- UI update dispatcher (Serial) ----------------------

void updateUI() {
  unsigned long nowMs = millis();
  if (nowMs - lastUiUpdateMs < UI_UPDATE_MS) return;
  lastUiUpdateMs = nowMs;

  switch (mode) {
    case MODE_BOOT:
      showBootScreen();
      break;
    case MODE_IDLE:
      showIdleScreen();
      break;
    case MODE_ACTIVE:
      showActiveScreen();
      break;
    case MODE_CAL_BASELINE:
      showCalBaselineScreen(calCollecting, calSamplesCollected);
      break;
    case MODE_CAL_MAX:
      showCalMaxScreen(calCollecting, calSamplesCollected);
      break;
    case MODE_ERROR:
      showErrorScreen();
      break;
    default:
      break;
  }
}

// ---------------------- State machine logic ----------------------

void handleModeTransitions() {
  switch (mode) {
    case MODE_BOOT:
      // After boot, go to IDLE
      mode = MODE_IDLE;
      break;

    case MODE_IDLE:
      // Go to calibration
      if (btnSelect.pressedEvent) {
        calCollecting = false;
        calAccum = 0;
        calSamplesCollected = 0;
        mode = MODE_CAL_BASELINE;
      }

      // Go to active control (only if calibration is valid)
      if (btnUp.pressedEvent) {
        mode = MODE_ACTIVE;
      }
      break;

    case MODE_ACTIVE:
      // Exit to IDLE
      if (btnSelect.pressedEvent) {
        mode = MODE_IDLE;
      }
      // If serious fault, go to ERROR
      if (emgFlatFault) {
        mode = MODE_ERROR;
        errorMessage = F("Flat EMG signal");
      }
      break;

    case MODE_CAL_BASELINE:
      if (!calCollecting && btnSelect.pressedEvent) {
        // Start baseline capture
        calCollecting = true;
        calAccum = 0;
        calSamplesCollected = 0;
      }

      // When CAL_SAMPLES complete, store baseline and move to max cal
      if (!calCollecting && calSamplesCollected >= CAL_SAMPLES) {
        if (calSamplesCollected > 0) {
          cal.baseline = (uint16_t)(calAccum / calSamplesCollected);
        } else {
          cal.baseline = (uint16_t)emgFiltered;
        }
        // Prepare for max phase
        calCollecting = false;
        calAccum = 0;
        calSamplesCollected = 0;
        mode = MODE_CAL_MAX;
      }
      break;

    case MODE_CAL_MAX:
      if (!calCollecting && btnSelect.pressedEvent) {
        // Start max capture
        calCollecting = true;
        calAccum = 0;
        calSamplesCollected = 0;
      }

      if (!calCollecting && calSamplesCollected >= CAL_SAMPLES) {
        if (calSamplesCollected > 0) {
          cal.max = (uint16_t)(calAccum / calSamplesCollected);
        } else {
          cal.max = (uint16_t)emgFiltered;
        }

        if (cal.max <= cal.baseline) {
          mode = MODE_ERROR;
          errorMessage = F("Calibration error: max <= baseline");
        } else {
          saveCalibration();
          mode = MODE_IDLE;
        }
      }
      break;

    case MODE_ERROR:
      // Allow reset back to IDLE with SELECT
      if (btnSelect.pressedEvent) {
        emgFlatFault = false;
        mode = MODE_IDLE;
      }
      break;

    default:
      break;
  }
}

// ---------------------- Setup ----------------------

void setup() {
  // Pins
  pinMode(EMG_PIN, INPUT);
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_SELECT_PIN, INPUT_PULLUP);

  // Serial
  Serial.begin(115200);
  while (!Serial) { ; } // wait for Serial on some boards
  Serial.println(F("EMG → Servo controller (Serial UI only) starting..."));

  // Servo
  servo.attach(SERVO_PIN);
  servoAngleCurrent = SERVO_MIN_ANGLE;
  servo.write((int)servoAngleCurrent);

  // Load calibration
  loadCalibration();

  // Initialize timing
  nextSampleTimeUs  = micros() + SAMPLE_PERIOD_US;
  lastServoUpdateMs = millis();
  lastUiUpdateMs    = 0;
  lastFlatCheckMs   = millis();
  lastFlatCheckValue = 0;

  // Initial EMG filtered
  emgFiltered = (float)analogRead(EMG_PIN);

  mode = MODE_BOOT;
}

// ---------------------- Main loop ----------------------

void loop() {
  // EMG sampling at fixed interval (approx)
  unsigned long nowUs = micros();
  if ((long)(nowUs - nextSampleTimeUs) >= 0) {
    nextSampleTimeUs += SAMPLE_PERIOD_US;
    sampleEMG();
  }

  // Buttons
  updateButtons();

  // State machine
  handleModeTransitions();

  // Servo
  updateServo();

  // Text UI
  updateUI();

  // Optional: extra debug (commented out)
  /*
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 500) {
    lastDbg = millis();
    Serial.print(F("EMG raw="));
    Serial.print(emgRaw);
    Serial.print(F(" filt="));
    Serial.print(emgFiltered);
    Serial.print(F(" norm="));
    Serial.print(emgNorm, 2);
    Serial.print(F(" angle="));
    Serial.println(servoAngleCurrent);
  }
  */
}
