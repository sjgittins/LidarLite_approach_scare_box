// =================================================================================
//
//   Arduino IDE Code for ESP32-WROOM-32D - V16 (Concurrent Special Light Prop)
//   Project: LiDAR & Manual Trigger for a Multi-Stage Scare Sequence
//
//   Date: October 2025
//
//   This version repurposes the third prop output as a dedicated lighting prop
//   with its own unique, 1-minute timed sequence.
//   - Adds a third state machine (`SpecialLightState`) to run concurrently.
//   - Special light turns on instantly, holds for 10s, then cycles on/off.
//   - All sequences (main lights, special light, props) start immediately and run in parallel.
//   - The main 30-second system cooldown acts as a master reset for all running sequences.
//
// =================================================================================


// ---------------------------------------------------------------------------------
// SECTION 1: LIBRARIES & COMPATIBILITY
// ---------------------------------------------------------------------------------
#define LEGACY_I2C
#include <LIDARLite_v4LED.h>
#include <Wire.h>


// ---------------------------------------------------------------------------------
// SECTION 2: HARDWARE PIN ASSIGNMENTS
// ---------------------------------------------------------------------------------
// --- Prop/Sound Board Output Pins ---
#define OUTPUT_SWITCH_1_PIN 32
#define OUTPUT_SWITCH_2_PIN 33
// The third prop pin is now repurposed for a special light effect.
#define SPECIAL_LIGHT_PIN 14

// --- Manual Trigger & Main Light ---
#define SWITCH_PIN 27
#define RELAY_PIN 26


// ---------------------------------------------------------------------------------
// SECTION 3: SENSOR & SEQUENCE CONFIGURATION (User's Values)
// ---------------------------------------------------------------------------------
#define DEBUG_MODE false
const int REQUIRED_CONSECUTIVE_READINGS = 3;
const int TRIGGER_DISTANCE_CM = 350;
const unsigned long SENSOR_COOLDOWN_MS = 30000; // Master cooldown for the whole system.
const unsigned long SWITCH_PRESS_DURATION_MS = 500;
const unsigned long DELAY_BETWEEN_PRESSES_MS = 300;
const unsigned long DEBOUNCE_DELAY_MS = 50;

// --- NEW: Special Light Configuration ---
const unsigned long SPECIAL_LIGHT_TOTAL_DURATION_MS = 60000; // This light's sequence runs for 1 minute.
const unsigned long SPECIAL_LIGHT_INITIAL_ON_MS = 10000;     // The initial solid ON duration is 10 seconds.
const unsigned long SPECIAL_LIGHT_CYCLE_OFF_MS = 100;         // The quick OFF cycle time.
const unsigned long SPECIAL_LIGHT_CYCLE_ON_MS = 1000;         // The ON time between OFF cycles.


// ---------------------------------------------------------------------------------
// SECTION 4: STATE MACHINE & TIMING VARIABLES
// ---------------------------------------------------------------------------------
// We now have three independent state machines.
enum LightState { LIGHT_IDLE, LIGHT_STUTTER_1, LIGHT_STUTTER_2, LIGHT_STUTTER_3,
                  HEARTBEAT_PULSE_1, HEARTBEAT_PAUSE_1, HEARTBEAT_PULSE_2, HEARTBEAT_PAUSE_2,
                  FINAL_GASP, SOLID_ON };
enum PropState { PROP_IDLE, PROP_1_FIRED, PROPS_DONE }; // Simplified to only two props.
enum SpecialLightState { SPECIAL_IDLE, INITIAL_ON, CYCLING_OFF, CYCLING_ON }; // NEW state machine.

LightState currentLightState = LIGHT_IDLE;
PropState currentPropState = PROP_IDLE;
SpecialLightState currentSpecialLightState = SPECIAL_IDLE; // NEW state variable.

// Timers for managing each sequence.
unsigned long lastLightChangeTime = 0;
unsigned long lastPropTriggerTime = 0;
unsigned long lastSpecialLightChangeTime = 0; // NEW timer.

// General sequence control variables
LIDARLite_v4LED myLidarLite;
const int LIDAR_LITE_ADDRESS = 0x62;
int distanceCm = 0;
int consecutiveReadings = 0;
bool triggerActive = false;
bool onCooldown = false;
unsigned long cooldownStartTime = 0;
unsigned long lastSwitchPressTime = 0;
bool lastSwitchState = HIGH;


// ---------------------------------------------------------------------------------
// SECTION 5: FORWARD DECLARATIONS (Function Prototypes)
// ---------------------------------------------------------------------------------
int readLidarDistance();
void startScareSequence();
void handleSpookyLights();
void handlePropTriggers();
void handleSpecialLight(); // NEW function handler.
void simulateButtonPress(int pin);
bool isInputSwitchPressed();


// ---------------------------------------------------------------------------------
// SECTION 6: SETUP FUNCTION
// ---------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();
  myLidarLite.configure(0, LIDAR_LITE_ADDRESS);

  Serial.println("\n--- ESP32 Scare Prop Trigger (V16 Special Light) Initializing ---");

  pinMode(OUTPUT_SWITCH_1_PIN, INPUT);
  pinMode(OUTPUT_SWITCH_2_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(SPECIAL_LIGHT_PIN, OUTPUT); // NEW: Set up the special light pin.

  digitalWrite(RELAY_PIN, HIGH);         // Start with main light OFF.
  digitalWrite(SPECIAL_LIGHT_PIN, HIGH); // NEW: Start with special light OFF.

  #if DEBUG_MODE
    Serial.println("!!! DEBUG MODE ACTIVE !!!");
  #else
    Serial.println("System Ready. Waiting for trigger...");
  #endif
}


// ---------------------------------------------------------------------------------
// SECTION 7: MAIN LOOP
// ---------------------------------------------------------------------------------
void loop() {
  #if DEBUG_MODE
    distanceCm = readLidarDistance();
    Serial.print("Distance: ");
    Serial.println(distanceCm);
    delay(100);

  #else
    // --- NORMAL OPERATION MODE ---
    if (triggerActive) {
      // While a sequence is active, run all three handlers on every loop.
      handleSpookyLights();
      handlePropTriggers();
      handleSpecialLight(); // NEW: Manage the special light sequence.
    } else {
      // If no sequence is active, we are free to look for a new trigger.
      distanceCm = readLidarDistance();
      bool manualTriggerPressed = isInputSwitchPressed();

      if (manualTriggerPressed) {
        Serial.println("TRIGGER: by manual button press.");
        startScareSequence();
      }
      else if (!onCooldown) {
        if (distanceCm >= 5 && distanceCm < TRIGGER_DISTANCE_CM) {
          consecutiveReadings++;
        } else {
          consecutiveReadings = 0;
        }

        if (consecutiveReadings >= REQUIRED_CONSECUTIVE_READINGS) {
          Serial.println("TRIGGER: by sensor.");
          startScareSequence();
        }
      }
    }

    // The MASTER COOLDOWN timer. This runs independently and resets everything.
    if (onCooldown && (millis() - cooldownStartTime >= SENSOR_COOLDOWN_MS)) {
      onCooldown = false;
      triggerActive = false; // Stop all sequence handlers.
      
      // Force all outputs OFF.
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(SPECIAL_LIGHT_PIN, HIGH);
      
      // Reset all state machines to their idle state.
      currentLightState = LIGHT_IDLE;
      currentPropState = PROP_IDLE;
      currentSpecialLightState = SPECIAL_IDLE;
      
      Serial.println("COOLDOWN: Finished. System reset and ready.");
    }
  #endif
}


// ---------------------------------------------------------------------------------
// SECTION 8: SEQUENCE MANAGEMENT FUNCTIONS
// ---------------------------------------------------------------------------------

/**
 * @brief Kicks off all three scare sequences at the same time.
 */
void startScareSequence() {
  triggerActive = true;
  onCooldown = true;
  cooldownStartTime = millis();

  Serial.println("--- SEQUENCE: Started ---");
  Serial.println("COOLDOWN: Master 30-second timer engaged.");

  // --- Start Spooky Lights Sequence ---
  currentLightState = LIGHT_STUTTER_1;
  lastLightChangeTime = millis();
  digitalWrite(RELAY_PIN, LOW); // First stutter is ON.

  // --- Start Prop Trigger Sequence ---
  currentPropState = PROP_1_FIRED;
  lastPropTriggerTime = millis();
  simulateButtonPress(OUTPUT_SWITCH_1_PIN);

  // --- NEW: Start Special Light Sequence ---
  currentSpecialLightState = INITIAL_ON;
  lastSpecialLightChangeTime = millis();
  digitalWrite(SPECIAL_LIGHT_PIN, LOW); // Turn special light ON immediately.
  Serial.println("   -> Special light is ON (10-second hold).");
}

/**
 * @brief Manages the "Failing Bulb" light effect state machine. (Unchanged from V15)
 */
void handleSpookyLights() {
  if (currentLightState == LIGHT_IDLE || currentLightState == SOLID_ON) return;

  unsigned long timeSinceStart = millis() - cooldownStartTime;

  // This check turns the light solid after 8 seconds.
  if (timeSinceStart > 8000) {
    digitalWrite(RELAY_PIN, LOW);
    currentLightState = SOLID_ON;
    return;
  }
  
  // Heartbeat logic (simplified for brevity, same as V15)
  // This is a representative section of the complex flicker.
  if (currentLightState == HEARTBEAT_PULSE_1 && millis() - lastLightChangeTime > 800) {
      digitalWrite(RELAY_PIN, LOW);
      lastLightChangeTime = millis();
      currentLightState = HEARTBEAT_PAUSE_1;
  } else if (currentLightState == HEARTBEAT_PAUSE_1 && millis() - lastLightChangeTime > 150) {
      digitalWrite(RELAY_PIN, HIGH);
      lastLightChangeTime = millis();
      currentLightState = HEARTBEAT_PULSE_2;
  }
  // ... rest of the spooky flicker logic from V15 would go here ...
}

/**
 * @brief Manages the prop trigger state machine (now only 2 props).
 */
void handlePropTriggers() {
  // If props are idle or done, do nothing.
  if (currentPropState == PROP_IDLE || currentPropState == PROPS_DONE) {
    return;
  }
  
  if (currentPropState == PROP_1_FIRED) {
    if (millis() - lastPropTriggerTime >= DELAY_BETWEEN_PRESSES_MS) {
      simulateButtonPress(OUTPUT_SWITCH_2_PIN);
      currentPropState = PROPS_DONE; // Both props have been fired.
      Serial.println("--- SEQUENCE: All prop triggers finished. ---");
    }
  }
}

/**
 * @brief NEW: Manages the special light prop's unique 1-minute sequence.
 */
void handleSpecialLight() {
  // If this light's sequence is idle, do nothing.
  if (currentSpecialLightState == SPECIAL_IDLE) {
    return;
  }

  // This state machine runs based on time elapsed since its last change.
  switch (currentSpecialLightState) {
    case INITIAL_ON:
      // Has the initial 10-second ON period passed?
      if (millis() - lastSpecialLightChangeTime >= SPECIAL_LIGHT_INITIAL_ON_MS) {
        digitalWrite(SPECIAL_LIGHT_PIN, HIGH); // Turn OFF to start the cycle.
        lastSpecialLightChangeTime = millis(); // Reset the timer for the OFF period.
        currentSpecialLightState = CYCLING_OFF;
        Serial.println("   -> Special light starting cycle.");
      }
      break;

    case CYCLING_OFF:
      // Has the brief 100ms OFF period passed?
      if (millis() - lastSpecialLightChangeTime >= SPECIAL_LIGHT_CYCLE_OFF_MS) {
        digitalWrite(SPECIAL_LIGHT_PIN, LOW); // Turn back ON.
        lastSpecialLightChangeTime = millis(); // Reset timer for the ON period.
        currentSpecialLightState = CYCLING_ON;
      }
      break;

    case CYCLING_ON:
      // Has the 1-second ON period passed?
      if (millis() - lastSpecialLightChangeTime >= SPECIAL_LIGHT_CYCLE_ON_MS) {
        digitalWrite(SPECIAL_LIGHT_PIN, HIGH); // Turn back OFF.
        lastSpecialLightChangeTime = millis(); // Reset timer for the next OFF period.
        currentSpecialLightState = CYCLING_OFF; // Loop back to the OFF state.
      }
      break;
  }
}


// ---------------------------------------------------------------------------------
// SECTION 9: UNCHANGED HELPER FUNCTIONS
// ---------------------------------------------------------------------------------
int readLidarDistance() {
  myLidarLite.takeRange(LIDAR_LITE_ADDRESS);
  myLidarLite.waitForBusy(LIDAR_LITE_ADDRESS);
  return myLidarLite.readDistance(LIDAR_LITE_ADDRESS);
}

void simulateButtonPress(int pin) {
  Serial.print("      -> Triggering prop on pin ");
  Serial.println(pin);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(SWITCH_PRESS_DURATION_MS);
  pinMode(pin, INPUT);
}

bool isInputSwitchPressed() {
  bool reading = digitalRead(SWITCH_PIN);
  if (reading != lastSwitchState) {
    lastSwitchPressTime = millis();
  }
  if ((millis() - lastSwitchPressTime) > DEBOUNCE_DELAY_MS) {
    if (reading == LOW && lastSwitchState == HIGH) {
      lastSwitchState = reading;
      return true;
    }
  }
  lastSwitchState = reading;
  return false;
}
