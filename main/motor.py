# =============================================================================
#  motor.py
# -----------------------------------------------------------------------------
#  Motor driver output stage for the line follower robot (LFR), targeting a
#  TB6612FNG dual H-bridge driving 2 DC gear motors from a 2S LiPo supply.
#
#  This file owns everything between "a desired PWM value exists" and
#  "the physical motor pins are driven": direction resolution, dead-zone
#  compensation, motor balancing, acceleration/deceleration rate limiting,
#  PWM saturation, and the hardware write itself. It also implements the
#  robot-wide safe stop / brake / coast / emergency stop primitives.
#
#  This file consumes leftTargetPWM / rightTargetPWM, which are assumed to
#  already be fully computed by Control.ino (PID mixing complete). It does
#  NOT compute PID, line position, error, or track classification - it only
#  shapes and outputs whatever PWM values it is handed.
#
#  Hard constraints followed:
#      - No sensor code, no PID computation, no control/track logic.
#      - No delay() and no millis() anywhere - every function here executes
#        in bounded, constant time suitable for a 1-2 kHz control loop.
#      - Fixed-width integer types used throughout for all PWM/state values.
#      - No dynamic memory; all state lives in fixed-size static structures.
# =============================================================================

import machine
import config

# #############################################################################
# #  MODULE-LOCAL CONFIGURATION
# #############################################################################
MOTOR_BALANCE_FACTOR_LEFT  = 1.00
MOTOR_BALANCE_FACTOR_RIGHT = 1.00

# #############################################################################
# #  MOTOR STATE STRUCTURE
# #############################################################################
class MotorDriveState:
    def __init__(self, currentPWM=0, targetPWM=0, outputPWM=0, direction=True):
        self.currentPWM = currentPWM
        self.targetPWM = targetPWM
        self.outputPWM = outputPWM
        self.direction = direction

leftDrive  = MotorDriveState()
rightDrive = MotorDriveState()

stagedLeftPWM  = 0
stagedRightPWM = 0

motorDriverEnabled = False

# Pin instances
pinAin1 = None
pinAin2 = None
pinPwmA = None
pinBin1 = None
pinBin2 = None
pinPwmB = None
pinStby = None

# #############################################################################
# #  initializeMotors()
# #############################################################################
def initializeMotors():
    global pinAin1, pinAin2, pinPwmA, pinBin1, pinBin2, pinPwmB, pinStby

    pinAin1 = machine.Pin(config.PIN_MOTOR_AIN1, machine.Pin.OUT)
    pinAin2 = machine.Pin(config.PIN_MOTOR_AIN2, machine.Pin.OUT)
    pinPwmA = machine.PWM(machine.Pin(config.PIN_MOTOR_PWMA))
    pinPwmA.freq(20000)

    pinBin1 = machine.Pin(config.PIN_MOTOR_BIN1, machine.Pin.OUT)
    pinBin2 = machine.Pin(config.PIN_MOTOR_BIN2, machine.Pin.OUT)
    pinPwmB = machine.PWM(machine.Pin(config.PIN_MOTOR_PWMB))
    pinPwmB.freq(20000)

    pinStby = machine.Pin(config.PIN_MOTOR_STBY, machine.Pin.OUT)

    pinAin1.value(0)
    pinAin2.value(0)
    pinPwmA.duty_u16(0)

    pinBin1.value(0)
    pinBin2.value(0)
    pinPwmB.duty_u16(0)

    pinStby.value(0)
    global motorDriverEnabled
    motorDriverEnabled = False

    disableMotorDriver()

# #############################################################################
# #  enableMotorDriver() / disableMotorDriver()
# #############################################################################
def enableMotorDriver():
    global motorDriverEnabled
    pinStby.value(1)
    motorDriverEnabled = True

def disableMotorDriver():
    global motorDriverEnabled, leftDrive, rightDrive, stagedLeftPWM, stagedRightPWM
    pinAin1.value(0)
    pinAin2.value(0)
    pinPwmA.duty_u16(0)

    pinBin1.value(0)
    pinBin2.value(0)
    pinPwmB.duty_u16(0)

    pinStby.value(0)
    motorDriverEnabled = False

    leftDrive  = MotorDriveState(0, 0, 0, True)
    rightDrive = MotorDriveState(0, 0, 0, True)

    stagedLeftPWM  = 0
    stagedRightPWM = 0

# #############################################################################
# #  setLeftMotorHardware() / setRightMotorHardware()
# #############################################################################
def setLeftMotorHardware(speed):
    if speed > 0:
        pinAin1.value(1)
        pinAin2.value(0)
        duty = speed
    elif speed < 0:
        pinAin1.value(0)
        pinAin2.value(1)
        duty = -speed
    else:
        pinAin1.value(0)
        pinAin2.value(0)
        duty = 0

    if duty > config.MOTOR_MAX_PWM:
        duty = config.MOTOR_MAX_PWM

    duty_16 = (duty * 65535) // 255
    pinPwmA.duty_u16(duty_16)

    config.leftMotor = speed
    config.leftMotorState.pwm = duty
    config.leftMotorState.speed = speed
    config.leftMotorState.direction = 1 if speed > 0 else (-1 if speed < 0 else 0)

def setRightMotorHardware(speed):
    if speed > 0:
        pinBin1.value(1)
        pinBin2.value(0)
        duty = speed
    elif speed < 0:
        pinBin1.value(0)
        pinBin2.value(1)
        duty = -speed
    else:
        pinBin1.value(0)
        pinBin2.value(0)
        duty = 0

    if duty > config.MOTOR_MAX_PWM:
        duty = config.MOTOR_MAX_PWM

    duty_16 = (duty * 65535) // 255
    pinPwmB.duty_u16(duty_16)

    config.rightMotor = speed
    config.rightMotorState.pwm = duty
    config.rightMotorState.speed = speed
    config.rightMotorState.direction = 1 if speed > 0 else (-1 if speed < 0 else 0)

def setMotorsHardware(left, right):
    setLeftMotorHardware(left)
    setRightMotorHardware(right)

# #############################################################################
# #  applyMotorBalance()
# #############################################################################
def applyMotorBalance():
    global stagedLeftPWM, stagedRightPWM
    fLeft  = float(config.leftTargetPWM) * MOTOR_BALANCE_FACTOR_LEFT
    fRight = float(config.rightTargetPWM) * MOTOR_BALANCE_FACTOR_RIGHT

    stagedLeftPWM  = int(fLeft)
    stagedRightPWM = int(fRight)

# #############################################################################
# #  applyDeadZone()
# #############################################################################
def applyDeadZone():
    global stagedLeftPWM, stagedRightPWM
    if stagedLeftPWM > 0:
        stagedLeftPWM += config.MOTOR_DEAD_ZONE
    elif stagedLeftPWM < 0:
        stagedLeftPWM -= config.MOTOR_DEAD_ZONE

    if stagedRightPWM > 0:
        stagedRightPWM += config.MOTOR_DEAD_ZONE
    elif stagedRightPWM < 0:
        stagedRightPWM -= config.MOTOR_DEAD_ZONE

# #############################################################################
# #  limitPWM()
# #############################################################################
def limitPWM():
    global stagedLeftPWM, stagedRightPWM
    if stagedLeftPWM > config.MOTOR_PWM_SATURATION:
        stagedLeftPWM = config.MOTOR_PWM_SATURATION
    elif stagedLeftPWM < -config.MOTOR_PWM_SATURATION:
        stagedLeftPWM = -config.MOTOR_PWM_SATURATION

    if stagedRightPWM > config.MOTOR_PWM_SATURATION:
        stagedRightPWM = config.MOTOR_PWM_SATURATION
    elif stagedRightPWM < -config.MOTOR_PWM_SATURATION:
        stagedRightPWM = -config.MOTOR_PWM_SATURATION

# #############################################################################
# #  limitAcceleration()
# #############################################################################
def limitAcceleration():
    global stagedLeftPWM, stagedRightPWM

    leftDelta = stagedLeftPWM - leftDrive.currentPWM
    if leftDelta > config.MOTOR_ACCEL_LIMIT:
        leftDelta = config.MOTOR_ACCEL_LIMIT
    elif leftDelta < -config.MOTOR_DECEL_LIMIT:
        leftDelta = -config.MOTOR_DECEL_LIMIT
    stagedLeftPWM = leftDrive.currentPWM + leftDelta

    rightDelta = stagedRightPWM - rightDrive.currentPWM
    if rightDelta > config.MOTOR_ACCEL_LIMIT:
        rightDelta = config.MOTOR_ACCEL_LIMIT
    elif rightDelta < -config.MOTOR_DECEL_LIMIT:
        rightDelta = -config.MOTOR_DECEL_LIMIT
    stagedRightPWM = rightDrive.currentPWM + rightDelta

# #############################################################################
# #  driveRobot()
# #############################################################################
def driveRobot():
    if not motorDriverEnabled:
        return

    leftDrive.targetPWM  = config.leftTargetPWM
    rightDrive.targetPWM = config.rightTargetPWM

    applyMotorBalance()
    applyDeadZone()
    limitPWM()
    limitAcceleration()

    leftDrive.outputPWM  = stagedLeftPWM
    rightDrive.outputPWM = stagedRightPWM

    leftDrive.currentPWM  = stagedLeftPWM
    rightDrive.currentPWM = stagedRightPWM

    leftDrive.direction  = (stagedLeftPWM >= 0)
    rightDrive.direction = (stagedRightPWM >= 0)

    setMotorsHardware(stagedLeftPWM, stagedRightPWM)

# #############################################################################
# #  stopRobot() / brakeRobot() / coastRobot() / emergencyStop()
# #############################################################################
def stopRobot():
    global stagedLeftPWM, stagedRightPWM
    stagedLeftPWM  = 0
    stagedRightPWM = 0

    leftDrive.targetPWM   = 0
    leftDrive.currentPWM  = 0
    leftDrive.outputPWM   = 0

    rightDrive.targetPWM  = 0
    rightDrive.currentPWM = 0
    rightDrive.outputPWM  = 0

    setMotorsHardware(0, 0)

def brakeRobot():
    global stagedLeftPWM, stagedRightPWM
    stagedLeftPWM  = 0
    stagedRightPWM = 0

    leftDrive.targetPWM   = 0
    leftDrive.currentPWM  = 0
    leftDrive.outputPWM   = 0

    rightDrive.targetPWM  = 0
    rightDrive.currentPWM = 0
    rightDrive.outputPWM  = 0

    pinAin1.value(1)
    pinAin2.value(1)
    pinPwmA.duty_u16(65535)

    pinBin1.value(1)
    pinBin2.value(1)
    pinPwmB.duty_u16(65535)

    config.leftMotor  = 0
    config.rightMotor = 0

def coastRobot():
    stopRobot()

def emergencyStop():
    disableMotorDriver()

# #############################################################################
# #  motorUpdate()
# #############################################################################
def motorUpdate():
    if motorDriverEnabled:
        driveRobot()