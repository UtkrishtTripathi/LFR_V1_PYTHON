# =============================================================================
#  control.py
# -----------------------------------------------------------------------------
#  High-level control logic for the line follower robot (LFR):
#
#      - Weighted-average line position estimation
#      - PID error / derivative / integral computation
#      - Automatic track segment classification (straight/curve/corner/etc.)
#      - Curve severity estimation
#      - Adaptive PID gain scheduling (gain sets from Config.ino)
#      - Speed planning (target cruising speed per track segment)
#      - Left/right motor speed calculation (PID mixing)
#
#  This file consumes already-processed sensor data via the Sensors.ino
#  interface (updateSensors(), getSensor(), isSensorActive()) and the shared
#  configuration (constants, PIDGains, SpeedProfile, TrackType, thresholds)
#  defined in Config.ino. It produces two numbers - the desired left and
#  right motor speeds - and stops there. It does NOT write to any pin, call
#  analogWrite(), or otherwise touch the motor driver; that is the sole
#  responsibility of Motor.ino.
#
#  Hard constraints followed:
#      - No motor PWM output, no analogWrite(), no sensor reading/MUX code,
#        no calibration, no telemetry, no EEPROM, no Serial commands, no
#        recovery maneuver logic.
#      - No delay() and no millis() anywhere in this file - the control loop
#        is purely computational and deterministic; scheduling of *when*
#        controlLoop() is called is Main.ino's responsibility.
#      - Fixed-width integer types used throughout. Floating point is used
#        in exactly one place (the weighted-average position calculation),
#        because the sensor weighting scheme is mathematically defined with
#        half-integer weights (e.g. -7.5 .. 7.5 for 16 sensors); every other
#        computation (error, derivative, integral, PID, speed planning,
#        motor mixing) is pure integer arithmetic.
# =============================================================================

import config
import sensors

# #############################################################################
# #  MODULE-LOCAL CONFIGURATION
# #############################################################################
TARGET_POSITION               = 0    # Desired line position: dead center of the sensor array.
INTEGRAL_CLAMP                = 8000 # Absolute clamp on the accumulated integral term (anti-windup).
PID_OUTPUT_CLAMP              = 255  # Absolute clamp on the final PID output (matches motor speed range).
DERIVATIVE_FILTER_OLD         = 3    # EMA-style derivative filter weight for the previous filtered value.
DERIVATIVE_FILTER_NEW         = 1    # EMA-style derivative filter weight for the newest raw derivative.

SEVERITY_ERROR_REFERENCE      = 1000 # abs(error) value considered "maximum severity" from error alone.
SEVERITY_DERIVATIVE_REFERENCE = 400  # abs(derivative) value considered "maximum severity" from derivative alone.

# #############################################################################
# #  MODULE-LOCAL STATE
# #############################################################################
activeSensorCount  = 0 # Number of sensors currently reporting "active" (on line).
lineWidth          = 0 # Estimated line width in sensor units (count of active contiguous sensors).
previousRawPosition= 0.0 # Saved raw floating point position for line-loss hold behavior.

# #############################################################################
# #  calculateLinePosition()
# #############################################################################
def calculateLinePosition():
    global activeSensorCount, lineWidth, previousRawPosition

    weightedSum = 0.0
    sensorSum   = 0.0
    activeSensorCount = 0

    halfCount = float(config.SENSOR_COUNT - 1) / 2.0

    for i in range(config.SENSOR_COUNT):
        val = sensors.getSensor(i)

        if val > config.SENSOR_ACTIVE_THRESHOLD:
            activeSensorCount += 1

        weight = (float(i) - halfCount) * 1000.0
        weightedSum += weight * float(val)
        sensorSum   += float(val)

    lineWidth = activeSensorCount

    if sensorSum > 0.0:
        rawPosition = weightedSum / sensorSum
        previousRawPosition = rawPosition
        config.linePosition = int(rawPosition)
    else:
        config.linePosition = int(previousRawPosition)

# #############################################################################
# #  calculateError()
# #############################################################################
def calculateError():
    config.lastLineError = config.lineError
    config.lineError     = config.linePosition - TARGET_POSITION

# #############################################################################
# #  calculateDerivative()
# #############################################################################
def calculateDerivative():
    rawDerivative = config.lineError - config.lastLineError

    filteredDerivative = (
        config.derivative * DERIVATIVE_FILTER_OLD + rawDerivative * DERIVATIVE_FILTER_NEW
    ) // (DERIVATIVE_FILTER_OLD + DERIVATIVE_FILTER_NEW)

    config.derivative = filteredDerivative

# #############################################################################
# #  calculateIntegral()
# #############################################################################
def calculateIntegral():
    if abs(config.lineError) < config.SENSOR_ACTIVE_THRESHOLD:
        config.integral += config.lineError

        if config.integral > INTEGRAL_CLAMP:
            config.integral = INTEGRAL_CLAMP
        elif config.integral < -INTEGRAL_CLAMP:
            config.integral = -INTEGRAL_CLAMP
    else:
        config.integral = 0

# #############################################################################
# #  estimateCurveSeverity()
# #############################################################################
def estimateCurveSeverity():
    absError      = abs(config.lineError)
    absDerivative = abs(config.derivative)

    errorContrib = (absError * 50) // SEVERITY_ERROR_REFERENCE
    if errorContrib > 50:
        errorContrib = 50

    derivContrib = (absDerivative * 50) // SEVERITY_DERIVATIVE_REFERENCE
    if derivContrib > 50:
        derivContrib = 50

    combined = errorContrib + derivContrib
    if combined > 100:
        combined = 100

    config.curveSeverity = combined
    return config.curveSeverity

# #############################################################################
# #  classifyTrack()
# #############################################################################
def classifyTrack():
    absError = abs(config.lineError)

    if activeSensorCount == 0:
        config.lineLost  = True
        config.trackType = config.TRACK_GAP
        return

    config.lineLost = False

    if activeSensorCount >= (config.SENSOR_COUNT - 2):
        config.trackType = config.TRACK_JUNCTION_CROSS
        return

    if activeSensorCount >= (config.SENSOR_COUNT // 2) and absError > 2000:
        config.trackType = config.TRACK_JUNCTION_T
        return

    if absError > 4500:
        config.trackType = config.TRACK_90_TURN
        return

    if config.curveSeverity >= config.CURVE_SEVERITY_HIGH:
        config.trackType = config.TRACK_SHARP_CURVE
        return

    if config.curveSeverity >= config.CURVE_SEVERITY_LOW:
        config.trackType = config.TRACK_CURVE
        return

    config.trackType = config.TRACK_STRAIGHT

# #############################################################################
# #  updatePIDGains()
# #############################################################################
def updatePIDGains():
    if config.trackType == config.TRACK_STRAIGHT:
        config.activeGains = config.GAIN_SET_STRAIGHT
    elif config.trackType == config.TRACK_CURVE:
        config.activeGains = config.GAIN_SET_CURVE
    elif config.trackType in (config.TRACK_SHARP_CURVE, config.TRACK_90_TURN):
        config.activeGains = config.GAIN_SET_SHARP
    else:
        config.activeGains = config.GAIN_SET_CURVE

# #############################################################################
# #  calculatePID()
# #############################################################################
def calculatePID():
    pTerm = (config.lineError * config.activeGains.Kp_num) // config.activeGains.Kp_den
    iTerm = (config.integral * config.activeGains.Ki_num) // config.activeGains.Ki_den
    dTerm = (config.derivative * config.activeGains.Kd_num) // config.activeGains.Kd_den

    output = pTerm + iTerm + dTerm

    if output > PID_OUTPUT_CLAMP:
        output = PID_OUTPUT_CLAMP
    elif output < -PID_OUTPUT_CLAMP:
        output = -PID_OUTPUT_CLAMP

    config.pidOutput = output

# #############################################################################
# #  calculateTargetSpeed()
# #############################################################################
def calculateTargetSpeed():
    prof = config.activeSpeedProfile

    if config.trackType == config.TRACK_STRAIGHT:
        targetSpeed = prof.straightSpeed
    elif config.trackType == config.TRACK_CURVE:
        targetSpeed = prof.curveSpeed
    elif config.trackType == config.TRACK_SHARP_CURVE:
        targetSpeed = prof.sharpCurveSpeed
    elif config.trackType == config.TRACK_90_TURN:
        targetSpeed = prof.turn90Speed
    elif config.trackType in (config.TRACK_JUNCTION_T, config.TRACK_JUNCTION_CROSS):
        targetSpeed = prof.curveSpeed
    elif config.trackType == config.TRACK_GAP:
        targetSpeed = prof.minSpeed
    else:
        targetSpeed = prof.baseSpeed

    reduction = (targetSpeed * config.curveSeverity) // 200
    targetSpeed -= reduction

    if targetSpeed < prof.minSpeed:
        targetSpeed = prof.minSpeed
    if targetSpeed > prof.maxSpeed:
        targetSpeed = prof.maxSpeed

    return targetSpeed

# #############################################################################
# #  calculateMotorSpeed()
# #############################################################################
def calculateMotorSpeed():
    base = calculateTargetSpeed()

    left  = base + config.pidOutput
    right = base - config.pidOutput

    if left > config.MOTOR_MAX_PWM:
        left = config.MOTOR_MAX_PWM
    if left < -config.MOTOR_MAX_PWM:
        left = -config.MOTOR_MAX_PWM

    if right > config.MOTOR_MAX_PWM:
        right = config.MOTOR_MAX_PWM
    if right < -config.MOTOR_MAX_PWM:
        right = -config.MOTOR_MAX_PWM

    config.leftTargetPWM  = left
    config.rightTargetPWM = right

# #############################################################################
# #  controlLoop()
# #############################################################################
def controlLoop():
    sensors.updateSensors()

    calculateLinePosition()
    calculateError()
    calculateDerivative()
    calculateIntegral()

    classifyTrack()
    estimateCurveSeverity()

    updatePIDGains()
    calculatePID()

    calculateTargetSpeed()
    calculateMotorSpeed()