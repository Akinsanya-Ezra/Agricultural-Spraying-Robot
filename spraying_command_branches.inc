// Documentation extract: branches from handlePiCommand(const char* command).
// These statements belong INSIDE that original handler. This is not a sketch.
// Shared mode/inhibit handling and navigation branches are intentionally omitted.
// sendAck(), sendNack(), stopMotors() and shared state come from the host firmware.

  if (strcmp(command, "HB") == 0) {
    lastHeartbeatMs = millis();
    heartbeatSeen = true;
    PiSerial.println("HB_ACK");
    return;
  }

  if (strcmp(command, "SPRAY,OFF") == 0) {
    sprayRequested = false;
    setPumpHardware(false);
    sendAck("SPRAY,OFF");
    return;
  }

  if (strcmp(command, "TARGET,WEED") == 0 || strcmp(command, "TARGET,MAIZE") == 0) {
    if (!heartbeatHealthy()) {
      sendNack("TARGET,UNAVAILABLE");
      return;
    }
    const SprayTarget target = strcmp(command, "TARGET,WEED") == 0 ? SprayTarget::WEED : SprayTarget::MAIZE;
    if (!setSprayTarget(target)) {
      sendNack("SERVO_UNAVAILABLE");
      return;
    }
    sendAck(command);
    return;
  }

  if (strcmp(command, "FLUID,SENSOR,1") == 0 || strcmp(command, "FLUID,SENSOR,0") == 0) {
    if (controlMode != ControlMode::MANUAL || !heartbeatHealthy()) {
      sendNack("FLUID,SENSOR,UNAVAILABLE");
      return;
    }
    sprayRequested = false;
    setPumpHardware(false);
    stopMotors();
    const bool enabled = command[13] == '1';
    if (enabled != fluidSensorEnabled) {
      fluidSensorEnabled = enabled;
      recoveringFromEmpty = fluidRecoveryRequired;
      fluidState = enabled ? FluidState::CHECKING : FluidState::BYPASSED;
      fluidStateStartedMs = millis();
      if (enabled) resetLevelFilter();
    }
    sendAck(command);
    return;
  }

  if (strcmp(command, "FLUID,RECOVER") == 0) {
    if (!heartbeatHealthy() || !fluidSensorEnabled || fluidState != FluidState::FLUID_AVAILABLE ||
      !rawFluidPresent() || !fluidRecoveryRequired) {
      sendNack("FLUID,RECOVER,UNAVAILABLE");
      return;
    }
    sprayRequested = false;
    setPumpHardware(false);
    stopMotors();
    fluidRecoveryRequired = false;
    recoveringFromEmpty = false;
    sendAck("FLUID,RECOVER");
    PiSerial.println("EVENT,FLUID_RECOVERY_ACKNOWLEDGED");
    return;
  }

  if (strcmp(command, "SPRAY,ON") == 0) {
    if (!fluidOperational()) {
      sendNack("SPRAY,TANK_UNAVAILABLE");
      return;
    }
    if (!nozzleSettled()) {
      sendNack("SPRAY,SERVO_NOT_READY");
      return;
    }
    if (!spraySafetyAllowsPump()) {
      sendNack("SPRAY,UNAVAILABLE");
      return;
    }
    sprayRequested = true;
    lastSprayRequestMs = millis();
    applySprayOutput();
    sendAck("SPRAY,ON");
    return;
  }
