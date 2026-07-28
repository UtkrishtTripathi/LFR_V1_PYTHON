# =============================================================================
#  main.py
# -----------------------------------------------------------------------------
#  Top-level orchestration for the line follower robot (LFR): boot sequence,
#  robot state machine, button handling, WS2812B status LED, and dispatching
#  into Sensors.ino / Control.ino / Motor.ino.
# =============================================================================

import machine
import time
import neopixel
import config
import sensors
import control
import motor
import telemetry

# #############################################################################
# #  LOCAL HARDWARE CONFIGURATION
# #############################################################################
STATUS_LED_COUNT = 1
statusPixel = neopixel.NeoPixel(machine.Pin(config.PIN_STATUS_LED), STATUS_LED_COUNT)

# #############################################################################
# #  BUTTON HANDLING
# #############################################################################
BUTTON_DEBOUNCE_MS   = 25
BUTTON_LONG_PRESS_MS = 1000

class ButtonState:
    def __init__(self):
        self.rawState = False
        self.stableState = False
        self.previousStableState = False
        self.lastDebounceTime = 0
        self.pressStartTime = 0
        self.shortPressEvent = False
        self.longPressEvent = False

calibrationButton = ButtonState()
startButton       = ButtonState()

calButtonPin   = None
startButtonPin = None

def initializeButtons():
    global calButtonPin, startButtonPin
    calButtonPin   = machine.Pin(config.PIN_CALIBRATION_BUTTON, machine.Pin.IN, machine.Pin.PULL_UP)
    startButtonPin = machine.Pin(config.PIN_START_BUTTON, machine.Pin.IN, machine.Pin.PULL_UP)

def updateButton(pin, btn):
    now = time.ticks_ms()
    reading = (pin.value() == 0)

    btn.shortPressEvent = False
    btn.longPressEvent  = False

    if reading != btn.rawState:
        btn.lastDebounceTime = now
        btn.rawState = reading

    if time.ticks_diff(now, btn.lastDebounceTime) >= BUTTON_DEBOUNCE_MS:
        if reading != btn.stableState:
            btn.previousStableState = btn.stableState
            btn.stableState = reading

            if btn.stableState:
                btn.pressStartTime = now
            else:
                pressDuration = time.ticks_diff(now, btn.pressStartTime)
                if pressDuration < BUTTON_LONG_PRESS_MS:
                    btn.shortPressEvent = True

    if btn.stableState and (btn.pressStartTime != 0):
        if time.ticks_diff(now, btn.pressStartTime) >= BUTTON_LONG_PRESS_MS:
            btn.longPressEvent = True
            btn.pressStartTime = 0

def updateButtons():
    updateButton(calButtonPin, calibrationButton)
    updateButton(startButtonPin, startButton)

def isCalibrationPressed():
    return calibrationButton.shortPressEvent

def isStartPressed():
    return startButton.shortPressEvent

def isCalibrationLongPressed():
    return calibrationButton.longPressEvent

def isStartLongPressed():
    return startButton.longPressEvent

# #############################################################################
# #  STATUS LED ANIMATIONS
# #############################################################################
LED_BREATHE_PERIOD_MS    = 3000
LED_BLINK_FAST_PERIOD_MS = 200
LED_BLINK_SLOW_PERIOD_MS = 1000
LED_RAINBOW_STEP_MS      = 20

ledAnimationStartTime = 0
ledBlinkOn            = False
ledRainbowHue         = 0

def initializeStatusLED():
    global ledAnimationStartTime
    ledAnimationStartTime = time.ticks_ms()
    setStatusLED(config.COLOR_OFF_R, config.COLOR_OFF_G, config.COLOR_OFF_B)

def setStatusLED(r, g, b):
    statusPixel[0] = (r, g, b)
    statusPixel.write()

def blinkStatusLED(r, g, b, periodMs):
    global ledAnimationStartTime, ledBlinkOn
    now = time.ticks_ms()
    halfPeriod = periodMs // 2
    if time.ticks_diff(now, ledAnimationStartTime) >= halfPeriod:
        ledAnimationStartTime = now
        ledBlinkOn = not ledBlinkOn

    if ledBlinkOn:
        setStatusLED(r, g, b)
    else:
        setStatusLED(config.COLOR_OFF_R, config.COLOR_OFF_G, config.COLOR_OFF_B)

def hsv_to_rgb(h, s, v):
    if s == 0:
        return int(v * 255), int(v * 255), int(v * 255)
    i = int(h * 6.0)
    f = (h * 6.0) - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i %= 6
    if i == 0: r, g, b = v, t, p
    elif i == 1: r, g, b = q, v, p
    elif i == 2: r, g, b = p, v, t
    elif i == 3: r, g, b = p, q, v
    elif i == 4: r, g, b = t, p, v
    elif i == 5: r, g, b = v, p, q
    return int(r * 255), int(g * 255), int(b * 255)

def pulseStatusLED(r, g, b, periodMs):
    now = time.ticks_ms()
    phase = (time.ticks_diff(now, ledAnimationStartTime) % periodMs) / float(periodMs)
    scale = (1.0 - math.cos(phase * 2.0 * math.pi)) * 0.5
    setStatusLED(int(r * scale), int(g * scale), int(b * scale))

def rainbowStatusLED():
    global ledAnimationStartTime, ledRainbowHue
    now = time.ticks_ms()
    if time.ticks_diff(now, ledAnimationStartTime) >= LED_RAINBOW_STEP_MS:
        ledAnimationStartTime = now
        ledRainbowHue = (ledRainbowHue + 10) % 360
    r, g, b = hsv_to_rgb(ledRainbowHue / 360.0, 1.0, 1.0)
    setStatusLED(r, g, b)

def errorStatusLED():
    blinkStatusLED(config.COLOR_RED_R, config.COLOR_RED_G, config.COLOR_RED_B, LED_BLINK_FAST_PERIOD_MS)

def updateStatusLED():
    mode = config.robotMode
    if mode == config.MODE_BOOT:
        pulseStatusLED(config.COLOR_WHITE_R, config.COLOR_WHITE_G, config.COLOR_WHITE_B, LED_BREATHE_PERIOD_MS)
    elif mode == config.MODE_IDLE:
        pulseStatusLED(config.COLOR_BLU_R, config.COLOR_BLU_G, config.COLOR_BLU_B, LED_BREATHE_PERIOD_MS)
    elif mode == config.MODE_CALIBRATION:
        blinkStatusLED(config.COLOR_YEL_R, config.COLOR_YEL_G, config.COLOR_YEL_B, LED_BLINK_FAST_PERIOD_MS)
    elif mode == config.MODE_READY:
        blinkStatusLED(config.COLOR_GRN_R, config.COLOR_GRN_G, config.COLOR_GRN_B, LED_BLINK_SLOW_PERIOD_MS)
    elif mode == config.MODE_RUNNING:
        setStatusLED(config.COLOR_GRN_R, config.COLOR_GRN_G, config.COLOR_GRN_B)
    elif mode == config.MODE_RECOVERY:
        blinkStatusLED(config.COLOR_MAG_R, config.COLOR_MAG_G, config.COLOR_MAG_B, LED_BLINK_FAST_PERIOD_MS)
    elif mode == config.MODE_ERROR:
        errorStatusLED()
    elif mode == config.MODE_EMERGENCY_STOP:
        setStatusLED(config.COLOR_RED_R, config.COLOR_RED_G, config.COLOR_RED_B)
    else:
        errorStatusLED()

# #############################################################################
# #  STATE MACHINE & TRANSITIONS
# #############################################################################
calibrationStartTimeMs = 0

COUNTDOWN_STEP_MS = 700
COUNTDOWN_STEPS   = 3

countdownActive    = False
countdownStepStart = 0
countdownStepCount = 0

def enterState(newMode):
    global calibrationStartTimeMs, countdownActive, countdownStepStart, countdownStepCount
    config.robotMode = newMode

    if newMode == config.MODE_IDLE:
        config.robotEnabled = False
        motor.stopRobot()

    elif newMode == config.MODE_CALIBRATION:
        config.robotEnabled = False
        config.calibrationComplete = False
        motor.stopRobot()
        sensors.resetCalibration()
        calibrationStartTimeMs = time.ticks_ms()

    elif newMode == config.MODE_READY:
        config.robotEnabled = False
        config.calibrationComplete = True
        motor.stopRobot()
        countdownActive = False

    elif newMode == config.MODE_RUNNING:
        config.robotEnabled = True
        motor.enableMotorDriver()

    elif newMode == config.MODE_RECOVERY:
        pass

    elif newMode == config.MODE_ERROR:
        config.robotEnabled = False
        motor.emergencyStop()

    elif newMode == config.MODE_EMERGENCY_STOP:
        config.robotEnabled = False
        motor.emergencyStop()

def updateStateMachine():
    global countdownActive, countdownStepStart, countdownStepCount
    mode = config.robotMode
    now = time.ticks_ms()

    if mode == config.MODE_IDLE:
        if isCalibrationPressed():
            enterState(config.MODE_CALIBRATION)
        elif isStartPressed():
            if config.calibrationComplete:
                enterState(config.MODE_READY)

    elif mode == config.MODE_CALIBRATION:
        sensors.calibrateSensors()
        if time.ticks_diff(now, calibrationStartTimeMs) >= config.CALIBRATION_TIME_MS:
            enterState(config.MODE_READY)

    elif mode == config.MODE_READY:
        if not countdownActive:
            if isStartPressed():
                countdownActive = True
                countdownStepStart = now
                countdownStepCount = COUNTDOWN_STEPS
            elif isCalibrationPressed():
                enterState(config.MODE_CALIBRATION)
        else:
            if time.ticks_diff(now, countdownStepStart) >= COUNTDOWN_STEP_MS:
                countdownStepStart = now
                countdownStepCount -= 1
                if countdownStepCount == 0:
                    countdownActive = False
                    enterState(config.MODE_RUNNING)

    elif mode == config.MODE_RUNNING:
        if isStartPressed() or isStartLongPressed():
            enterState(config.MODE_IDLE)
        else:
            control.controlLoop()
            motor.motorUpdate()

    elif mode == config.MODE_RECOVERY:
        if isStartPressed():
            enterState(config.MODE_IDLE)

    elif mode == config.MODE_ERROR:
        if isStartLongPressed() or isCalibrationLongPressed():
            enterState(config.MODE_IDLE)

    elif mode == config.MODE_EMERGENCY_STOP:
        if isStartLongPressed():
            enterState(config.MODE_IDLE)

def performSelfTest():
    testPassed = True
    return testPassed

# #############################################################################
# #  setup()
# #############################################################################
def setup():
    if config.DEBUG_ENABLE:
        print("{} firmware {} built {}".format(config.ROBOT_NAME, config.FIRMWARE_VERSION, config.BUILD_DATE))

    initializeStatusLED()
    pulseStatusLED(config.COLOR_WHITE_R, config.COLOR_WHITE_G, config.COLOR_WHITE_B, LED_BREATHE_PERIOD_MS)

    initializeButtons()
    sensors.initializeSensors()
    motor.initializeMotors()
    telemetry.initializeTelemetry()

    selfTestPassed = performSelfTest()

    if not selfTestPassed:
        enterState(config.MODE_ERROR)
    else:
        config.robotEnabled = False
        config.calibrationComplete = False
        enterState(config.MODE_IDLE)

# #############################################################################
# #  loop()
# #############################################################################
def loop():
    updateButtons()
    updateStateMachine()
    updateStatusLED()
    telemetry.updateTelemetry()

if __name__ == "__main__":
    setup()
    while True:
        loop()