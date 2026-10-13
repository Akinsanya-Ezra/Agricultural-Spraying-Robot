// SPRAYING SUBSYSTEM DOCUMENTATION EXCERPT
// Extracted from the integrated controller supplied by Akinsanya Ezra.
// Not standalone firmware. Do not upload this excerpt to the robot.
// Navigation, perception and motor-control implementations are excluded.
// See README.md for dependencies and safety/configuration limitations.

#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <math.h>
#include <string.h>

// Dependencies retained in the complete robot firmware, not implemented here.
extern HardwareSerial PiSerial;
extern bool writePulse(uint8_t channel, int pulseUs);
extern void stopMotors();

constexpr int PI_RX_PIN = 18;
constexpr int PI_TX_PIN = 17;
constexpr uint32_t PI_BAUD = 230400;
constexpr uint32_t PI_HEARTBEAT_TIMEOUT_MS = 1000;
constexpr int NOZZLE_SERVO_PIN = 14;
constexpr int PUMP_RELAY_PIN = 13;
constexpr int LEVEL_SENSOR_PIN = 12;
constexpr uint8_t NOZZLE_CHANNEL = 4;
constexpr uint32_t PWM_FREQUENCY_HZ = 50;
constexpr uint8_t PWM_RESOLUTION_BITS = 14;
constexpr uint32_t PWM_PERIOD_US = 20000;
constexpr uint32_t PWM_STEPS = 1UL << PWM_RESOLUTION_BITS;
constexpr uint32_t SPRAY_LEASE_MS = 500;
constexpr bool PUMP_ACTIVE_LOW = false;
constexpr float HORIZONTAL_ANGLE_DEG = 97.6f;
constexpr float VERTICAL_ANGLE_DEG = 7.6f;
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr uint32_t NOZZLE_SETTLE_MS = 1000;
constexpr bool DEFAULT_FLUID_SENSOR_ENABLED = false;
constexpr int LEVEL_THRESHOLD = 500;
constexpr uint32_t LEVEL_SAMPLE_MS = 20;
constexpr uint32_t LEVEL_DEBOUNCE_MS = 300;
constexpr uint32_t DRY_CONFIRM_MS = 20000;
constexpr uint32_t FLUID_RECOVERY_STABLE_MS = 2000;
constexpr uint32_t SPRAY_TELEMETRY_INTERVAL_MS = 250;

enum class ControlMode : uint8_t { STOPPED, MANUAL, AUTO };
enum class FluidState : uint8_t {
  BYPASSED, CHECKING, FLUID_AVAILABLE, EMPTY_PENDING, EMPTY_CONFIRMED, RECOVERY_PENDING };
enum class SprayTarget : uint8_t { WEED, MAIZE };

ControlMode controlMode = ControlMode::STOPPED;
FluidState fluidState = FluidState::BYPASSED;
SprayTarget sprayTarget = SprayTarget::WEED;
bool autoInhibited = true;
bool heartbeatSeen = false;
bool pwmFault = false;
uint32_t lastHeartbeatMs = 0;
bool nozzleAttached = false;
bool pumpOn = false;
bool sprayRequested = false;
uint32_t lastSprayRequestMs = 0;
bool nozzleCommanded = false;
bool nozzleSettling = false;
uint32_t nozzleReadyAtMs = 0;
int nozzlePulseUs = 1500;
bool fluidSensorEnabled = DEFAULT_FLUID_SENSOR_ENABLED;
bool fluidRecoveryRequired = false;
bool recoveringFromEmpty = false;
bool levelKnown = false;
bool levelCandidateDry = true;
bool levelFilteredDry = true;
int levelReading = 1023;
uint32_t levelCandidateSinceMs = 0;
uint32_t lastLevelSampleMs = 0;
uint32_t fluidStateStartedMs = 0;
uint32_t lastSprayTelemetryMs = 0;

const char* fluidStateName() {
  switch (fluidState) {
    case FluidState::BYPASSED: return "bypassed";
    case FluidState::CHECKING: return "checking";
    case FluidState::FLUID_AVAILABLE: return "fluid_available";
    case FluidState::EMPTY_PENDING: return "empty_pending";
    case FluidState::EMPTY_CONFIRMED: return "empty_confirmed";
    case FluidState::RECOVERY_PENDING: return "recovery_pending";
  }
  return "checking";
}

const char* targetName() { return sprayTarget == SprayTarget::WEED ? "weed" : "maize"; }

const char* orientationName() { return sprayTarget == SprayTarget::WEED ? "horizontal" : "vertical"; }

bool heartbeatHealthy() {
  return heartbeatSeen && uint32_t(millis() - lastHeartbeatMs) <= PI_HEARTBEAT_TIMEOUT_MS;
}

uint32_t pulseToDuty(int pulseUs) {
  pulseUs = constrain(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
  return (uint32_t(pulseUs) * PWM_STEPS + PWM_PERIOD_US / 2) / PWM_PERIOD_US;
}

void setPumpHardware(bool enabled) {
  if (pwmFault) enabled = false;
  pumpOn = enabled;
  const bool outputHigh = PUMP_ACTIVE_LOW ? !enabled : enabled;
  digitalWrite(PUMP_RELAY_PIN, outputHigh ? HIGH : LOW);
}

bool nozzleSettled() { return nozzleAttached && nozzleCommanded && !nozzleSettling; }

void serviceNozzle() {
  if (nozzleSettling && static_cast<int32_t>(millis() - nozzleReadyAtMs) >= 0) { nozzleSettling = false; }
}

bool setSprayTarget(SprayTarget target) {
  sprayRequested = false;
  setPumpHardware(false);
  if (!nozzleAttached || pwmFault) return false;
  if (nozzleCommanded && target == sprayTarget) { return true; }
  sprayTarget = target;
  const float angle = target == SprayTarget::WEED ? HORIZONTAL_ANGLE_DEG : VERTICAL_ANGLE_DEG;
  nozzlePulseUs = int(lroundf( SERVO_MIN_US + angle * (SERVO_MAX_US - SERVO_MIN_US) / 180.0f));
  if (!writePulse(NOZZLE_CHANNEL, nozzlePulseUs)) return false;
  nozzleCommanded = true;
  nozzleSettling = true;
  nozzleReadyAtMs = millis() + NOZZLE_SETTLE_MS;
  return true;
}

void resetLevelFilter() {
  levelReading = analogRead(LEVEL_SENSOR_PIN);
  levelCandidateDry = levelReading >= LEVEL_THRESHOLD;
  levelFilteredDry = levelCandidateDry;
  levelKnown = false;
  levelCandidateSinceMs = millis();
  lastLevelSampleMs = millis();
}

void updateLevelFilter() {
  const uint32_t now = millis();
  if (uint32_t(now - lastLevelSampleMs) < LEVEL_SAMPLE_MS) return;
  lastLevelSampleMs = now;
  levelReading = analogRead(LEVEL_SENSOR_PIN);
  const bool rawDry = levelReading >= LEVEL_THRESHOLD;
  if (rawDry != levelCandidateDry) {
    levelCandidateDry = rawDry;
    levelCandidateSinceMs = now;
  }
  if ((!levelKnown || levelFilteredDry != levelCandidateDry) && uint32_t(now - levelCandidateSinceMs) >=
    LEVEL_DEBOUNCE_MS) {
    levelFilteredDry = levelCandidateDry;
    levelKnown = true;
  }
}

bool rawFluidPresent() { return levelKnown && !levelFilteredDry; }

bool fluidAvailableBeforeRecoveryAck() {
  return !fluidSensorEnabled || fluidState == FluidState::FLUID_AVAILABLE;
}

bool fluidOperational() {
  if (!fluidSensorEnabled) return true;
  return fluidState == FluidState::FLUID_AVAILABLE && !fluidRecoveryRequired;
}

bool fluidEmptyConfirmed() { return fluidSensorEnabled && fluidState == FluidState::EMPTY_CONFIRMED; }

void updateFluidState() {
  updateLevelFilter();
  if (!fluidSensorEnabled) {
    fluidState = FluidState::BYPASSED;
    return;
  }
  if (!levelKnown) {
    fluidState = FluidState::CHECKING;
    return;
  }
  const uint32_t now = millis();
  const bool present = rawFluidPresent();
  switch (fluidState) {
    case FluidState::BYPASSED:
    case FluidState::CHECKING:
    recoveringFromEmpty = fluidRecoveryRequired;
    fluidState = present ? FluidState::RECOVERY_PENDING : FluidState::EMPTY_PENDING;
    fluidStateStartedMs = now;
    break;
    case FluidState::FLUID_AVAILABLE:
    if (!present) {
      sprayRequested = false;
      setPumpHardware(false);
      stopMotors();
      fluidState = FluidState::EMPTY_PENDING;
      fluidStateStartedMs = now;
    }
    break;
    case FluidState::EMPTY_PENDING:
    sprayRequested = false;
    setPumpHardware(false);
    stopMotors();
    if (present) {
      fluidState = FluidState::FLUID_AVAILABLE;
      recoveringFromEmpty = false;
      fluidStateStartedMs = now;
    } else if (uint32_t(now - fluidStateStartedMs) >= DRY_CONFIRM_MS) {
      fluidState = FluidState::EMPTY_CONFIRMED;
      fluidRecoveryRequired = true;
      recoveringFromEmpty = true;
      controlMode = ControlMode::STOPPED;
      PiSerial.println("EVENT,FLUID_EMPTY");
    }
    break;
    case FluidState::EMPTY_CONFIRMED:
    sprayRequested = false;
    setPumpHardware(false);
    stopMotors();
    if (present) {
      fluidState = FluidState::RECOVERY_PENDING;
      recoveringFromEmpty = true;
      fluidStateStartedMs = now;
    }
    break;
    case FluidState::RECOVERY_PENDING:
    sprayRequested = false;
    setPumpHardware(false);
    stopMotors();
    if (!present) {
      fluidState = recoveringFromEmpty ? FluidState::EMPTY_CONFIRMED : FluidState::EMPTY_PENDING;
      fluidStateStartedMs = now;
    } else if (uint32_t(now - fluidStateStartedMs) >= FLUID_RECOVERY_STABLE_MS) {
      const bool recovered = recoveringFromEmpty;
      fluidState = FluidState::FLUID_AVAILABLE;
      recoveringFromEmpty = false;
      if (recovered) PiSerial.println("EVENT,FLUID_RECOVERED");
    }
    break;
  }
}

bool sprayLeaseFresh() { return sprayRequested && uint32_t(millis() - lastSprayRequestMs) < SPRAY_LEASE_MS; }

bool spraySafetyAllowsPump() {
  const bool activeMode = controlMode == ControlMode::MANUAL || controlMode == ControlMode::AUTO;
  return activeMode && !autoInhibited && heartbeatHealthy() && fluidOperational() && nozzleSettled() &&
    !pwmFault;
}

void applySprayOutput() {
  if (!sprayLeaseFresh()) sprayRequested = false;
  setPumpHardware(sprayRequested && spraySafetyAllowsPump());
}

void sendSprayAndFluidTelemetry() {
  const uint32_t now = millis();
  if (uint32_t(now - lastSprayTelemetryMs) < SPRAY_TELEMETRY_INTERVAL_MS) return;
  lastSprayTelemetryMs = now;
  PiSerial.println("PROTOCOL,2");
  PiSerial.print("SPRAY,");
  PiSerial.print(targetName()); PiSerial.print(',');
  PiSerial.print(orientationName()); PiSerial.print(',');
  PiSerial.print(nozzleAttached && !pwmFault ? 1 : 0); PiSerial.print(',');
  PiSerial.print(nozzleSettled() ? 1 : 0); PiSerial.print(',');
  PiSerial.print(pumpOn ? 1 : 0); PiSerial.print(',');
  PiSerial.println(sprayLeaseFresh() ? 1 : 0);
  PiSerial.print("FLUID,");
  PiSerial.print(fluidStateName()); PiSerial.print(',');
  PiSerial.print(fluidSensorEnabled ? 1 : 0); PiSerial.print(',');
  PiSerial.print(rawFluidPresent() ? 1 : 0); PiSerial.print(',');
  PiSerial.print(fluidAvailableBeforeRecoveryAck() ? 1 : 0); PiSerial.print(',');
  PiSerial.print(fluidRecoveryRequired ? 1 : 0); PiSerial.print(',');
  PiSerial.println(fluidEmptyConfirmed() ? 1 : 0);
}
