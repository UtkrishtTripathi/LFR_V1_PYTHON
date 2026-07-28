# =============================================================================
#  telemetry.py
# -----------------------------------------------------------------------------
#  Passive serial diagnostics and telemetry module for the line follower
#  robot (LFR).
# =============================================================================

import sys
import select
import time
import config
import sensors
import control
import motor

# #############################################################################
# #  TELEMETRY MODES
# #############################################################################
TELEMETRY_OFF         = 0
TELEMETRY_RAW         = 1
TELEMETRY_NORMALIZED  = 2
TELEMETRY_FILTERED    = 3
TELEMETRY_CALIBRATION = 4
TELEMETRY_PID         = 5
TELEMETRY_MOTOR       = 6
TELEMETRY_TRACK       = 7
TELEMETRY_PERFORMANCE = 8
TELEMETRY_ALL         = 9

# #############################################################################
# #  MODULE CONFIGURATION
# #############################################################################
TELEMETRY_BAUD_RATE       = 115200
TELEMETRY_PERIOD_MS       = 100
LOOP_OVERRUN_THRESHOLD_US = 1000
SENSOR_DISCONNECTED_ADC   = 5
SENSOR_SATURATED_MARGIN   = 5
TELEMETRY_CSV_MODE        = False

# #############################################################################
# #  MODULE STATE
# #############################################################################
telemetryEnabled     = False
telemetryMode        = TELEMETRY_OFF
lastTelemetryPrintMs = 0
serialStreamMuted    = False

lastLoopTimestampUs = 0
currentLoopTimeUs   = 0
minLoopTimeUs       = 0xFFFFFFFF
maxLoopTimeUs       = 0
totalLoopTimeUs     = 0
loopSampleCount     = 0

maxPIDOutputSeen   = 0
maxErrorSeen       = 0
maxSpeedSeen       = 0
maxPWMSeen         = 0
lostLineCount      = 0
junctionCount      = 0

previousTrackType     = config.TRACK_STRAIGHT
previousRobotEnabled  = False
raceTimerStartMs      = 0
raceTimerElapsedMs    = 0
calibrationDurationMs = 0

serialBuffer = ""

# #############################################################################
# #  initializeTelemetry()
# #############################################################################
def initializeTelemetry():
    global telemetryEnabled, telemetryMode, lastTelemetryPrintMs, serialStreamMuted
    global lastLoopTimestampUs, currentLoopTimeUs, minLoopTimeUs, maxLoopTimeUs, totalLoopTimeUs, loopSampleCount
    global maxPIDOutputSeen, maxErrorSeen, maxSpeedSeen, maxPWMSeen, lostLineCount, junctionCount
    global previousTrackType, previousRobotEnabled, raceTimerStartMs, raceTimerElapsedMs, calibrationDurationMs

    telemetryEnabled     = False
    telemetryMode        = TELEMETRY_OFF
    lastTelemetryPrintMs = 0
    serialStreamMuted    = False

    lastLoopTimestampUs = time.ticks_us()
    currentLoopTimeUs   = 0
    minLoopTimeUs       = 0xFFFFFFFF
    maxLoopTimeUs       = 0
    totalLoopTimeUs     = 0
    loopSampleCount     = 0

    maxPIDOutputSeen = 0
    maxErrorSeen     = 0
    maxSpeedSeen     = 0
    maxPWMSeen       = 0
    lostLineCount    = 0
    junctionCount    = 0

    previousTrackType    = config.TRACK_STRAIGHT
    previousRobotEnabled = False
    raceTimerStartMs     = 0
    raceTimerElapsedMs   = 0
    calibrationDurationMs= 0

def enableTelemetry(mode=TELEMETRY_ALL):
    global telemetryEnabled, telemetryMode
    telemetryEnabled = True
    telemetryMode    = mode

def disableTelemetry():
    global telemetryEnabled, telemetryMode
    telemetryEnabled = False
    telemetryMode    = TELEMETRY_OFF

def setTelemetryMode(mode):
    global telemetryMode
    telemetryMode = mode

def isTelemetryEnabled():
    return telemetryEnabled

def muteSerialStream(mute):
    global serialStreamMuted
    serialStreamMuted = mute

def isSerialStreamMuted():
    return serialStreamMuted

# #############################################################################
# #  updateLoopTiming()
# #############################################################################
def updateLoopTiming():
    global lastLoopTimestampUs, currentLoopTimeUs, minLoopTimeUs, maxLoopTimeUs, totalLoopTimeUs, loopSampleCount
    nowUs = time.ticks_us()
    if lastLoopTimestampUs != 0:
        elapsed = time.ticks_diff(nowUs, lastLoopTimestampUs)
        if elapsed > 0:
            currentLoopTimeUs = elapsed
            if currentLoopTimeUs < minLoopTimeUs:
                minLoopTimeUs = currentLoopTimeUs
            if currentLoopTimeUs > maxLoopTimeUs:
                maxLoopTimeUs = currentLoopTimeUs
            totalLoopTimeUs += currentLoopTimeUs
            loopSampleCount += 1
            config.diagnostics.loopTimeUs    = currentLoopTimeUs
            config.diagnostics.minLoopTimeUs = minLoopTimeUs
            config.diagnostics.maxLoopTimeUs = maxLoopTimeUs

    lastLoopTimestampUs = nowUs

# #############################################################################
# #  updateStatistics()
# #############################################################################
def updateStatistics():
    global maxPIDOutputSeen, maxErrorSeen, maxSpeedSeen, maxPWMSeen, lostLineCount, junctionCount
    global previousTrackType, previousRobotEnabled, raceTimerStartMs, raceTimerElapsedMs

    absPID = abs(config.pidOutput)
    if absPID > maxPIDOutputSeen:
        maxPIDOutputSeen = absPID

    absErr = abs(config.lineError)
    if absErr > maxErrorSeen:
        maxErrorSeen = absErr

    absLeft  = abs(config.leftMotor)
    absRight = abs(config.rightMotor)
    if absLeft > maxPWMSeen:
        maxPWMSeen = absLeft
    if absRight > maxPWMSeen:
        maxPWMSeen = absRight

    if config.trackType == config.TRACK_GAP and previousTrackType != config.TRACK_GAP:
        lostLineCount += 1

    if config.trackType in (config.TRACK_JUNCTION_T, config.TRACK_JUNCTION_CROSS) and previousTrackType not in (config.TRACK_JUNCTION_T, config.TRACK_JUNCTION_CROSS):
        junctionCount += 1

    previousTrackType = config.trackType

    nowMs = time.ticks_ms()
    if config.robotEnabled and not previousRobotEnabled:
        raceTimerStartMs = nowMs
        raceTimerElapsedMs = 0
    elif config.robotEnabled:
        raceTimerElapsedMs = time.ticks_diff(nowMs, raceTimerStartMs)

    previousRobotEnabled = config.robotEnabled

# #############################################################################
# #  PRINT FUNCTIONS
# #############################################################################
def sendTelemetrySerial():
    return telemetryEnabled and not serialStreamMuted

def printRawSensors():
    if not telemetryEnabled: return
    print("RAW:")
    for i in range(config.SENSOR_COUNT):
        print(" S{}: {}".format(i, config.sensor.raw[i]))

def printNormalizedSensors():
    if not telemetryEnabled: return
    print("NORM:")
    for i in range(config.SENSOR_COUNT):
        print(" S{}: {}".format(i, config.sensor.normalized[i]))

def printFilteredSensors():
    if not telemetryEnabled: return
    print("FILTER:")
    for i in range(config.SENSOR_COUNT):
        print(" S{}: {}".format(i, config.sensor.filtered[i]))

def printCalibration():
    if not telemetryEnabled: return
    print("CALIB:")
    for i in range(config.SENSOR_COUNT):
        print(" S{}: Min={} Max={}".format(i, config.sensor.minimum[i], config.sensor.maximum[i]))

def printPID():
    if not telemetryEnabled: return
    print("PID:")
    print(" Position: {}".format(config.linePosition))
    print(" Error:    {}".format(config.lineError))
    print(" Integral: {}".format(config.integral))
    print(" Deriv:    {}".format(config.derivative))
    print(" Output:   {}".format(config.pidOutput))

def printMotors():
    if not telemetryEnabled: return
    print("MOTORS:")
    print(" Left Target PWM:  {}".format(config.leftTargetPWM))
    print(" Right Target PWM: {}".format(config.rightTargetPWM))
    print(" Left Output PWM:  {}".format(config.leftMotor))
    print(" Right Output PWM: {}".format(config.rightMotor))
    print(" Left Direction:   {}".format("FORWARD" if config.leftMotor >= 0 else "REVERSE"))
    print(" Right Direction:  {}".format("FORWARD" if config.rightMotor >= 0 else "REVERSE"))

def getTrackTypeString(ttype):
    if ttype == config.TRACK_STRAIGHT: return "STRAIGHT"
    if ttype == config.TRACK_CURVE: return "CURVE"
    if ttype == config.TRACK_SHARP_CURVE: return "SHARP_CURVE"
    if ttype == config.TRACK_90_TURN: return "90_TURN"
    if ttype == config.TRACK_JUNCTION_T: return "JUNCTION_T"
    if ttype == config.TRACK_JUNCTION_CROSS: return "JUNCTION_CROSS"
    if ttype == config.TRACK_GAP: return "GAP"
    return "UNKNOWN"

def getRobotModeString(mode):
    if mode == config.MODE_BOOT: return "BOOT"
    if mode == config.MODE_IDLE: return "IDLE"
    if mode == config.MODE_CALIBRATION: return "CALIBRATION"
    if mode == config.MODE_READY: return "READY"
    if mode == config.MODE_RUNNING: return "RUNNING"
    if mode == config.MODE_RECOVERY: return "RECOVERY"
    if mode == config.MODE_ERROR: return "ERROR"
    if mode == config.MODE_EMERGENCY_STOP: return "EMERGENCY_STOP"
    return "UNKNOWN"

def printTrackInformation():
    if not telemetryEnabled: return
    print("TRACK:")
    print(" Type:     {}".format(getTrackTypeString(config.trackType)))
    print(" Severity: {}".format(config.curveSeverity))
    print(" Line Lost:{}".format("YES" if config.lineLost else "NO"))

def printRobotState():
    if not telemetryEnabled: return
    print("STATE:")
    print(" Mode:        {}".format(getRobotModeString(config.robotMode)))
    print(" Enabled:     {}".format("YES" if config.robotEnabled else "NO"))
    print(" Calibrated:  {}".format("YES" if config.calibrationComplete else "NO"))
    print(" Race Time:   {} ms".format(raceTimerElapsedMs))

def printLoopStatistics():
    if not telemetryEnabled: return
    avg = (totalLoopTimeUs // loopSampleCount) if loopSampleCount > 0 else 0
    print("LOOP:")
    print(" Current: {} us".format(currentLoopTimeUs))
    print(" Min:     {} us".format(minLoopTimeUs if minLoopTimeUs != 0xFFFFFFFF else 0))
    print(" Max:     {} us".format(maxLoopTimeUs))
    print(" Avg:     {} us".format(avg))

def printMemoryUsage():
    if not telemetryEnabled: return
    import gc
    print("MEMORY:")
    print(" Free:  {} bytes".format(gc.mem_free()))
    print(" Alloc: {} bytes".format(gc.mem_alloc()))

def printBatteryStatus():
    if not telemetryEnabled: return
    print("BATTERY:")
    print(" Status: Not Monitored")

def printPerformance():
    if not telemetryEnabled: return
    print("PERF:")
    print(" Max Error:  {}".format(maxErrorSeen))
    print(" Max PID:    {}".format(maxPIDOutputSeen))
    print(" Max PWM:    {}".format(maxPWMSeen))
    print(" Line Lost:  {} times".format(lostLineCount))
    print(" Junctions:  {}".format(junctionCount))

def printConfig():
    if not telemetryEnabled: return
    print("CONFIG:")
    print(" Sensors:   {}".format(config.SENSOR_COUNT))
    print(" Base Speed:{}".format(config.DEFAULT_SPEED_PROFILE.baseSpeed))

def runDiagnostics():
    if currentLoopTimeUs > LOOP_OVERRUN_THRESHOLD_US:
        if sendTelemetrySerial():
            print("WARN: Loop overrun detected: {} us".format(currentLoopTimeUs))

def printHelp():
    print("=== LFR Telemetry Commands ===")
    print(" raw         - print raw ADC values")
    print(" norm        - print normalized values")
    print(" filter      - print filtered values")
    print(" calib       - print min/max calibration bounds")
    print(" pid         - print PID terms")
    print(" motors      - print motor PWM outputs")
    print(" track       - print track classification")
    print(" state       - print robot operational state")
    print(" loop        - print loop timing statistics")
    print(" memory      - print heap memory usage")
    print(" battery     - print battery status")
    print(" perf        - print maximum performance metrics")
    print(" config      - print firmware configuration")
    print(" all         - print all telemetry parameters")
    print(" off         - disable periodic telemetry output")
    print(" start       - enable telemetry output")
    print(" clear       - clear running statistics")
    print(" status      - print telemetry system status")
    print(" mute        - mute serial stream output")
    print(" unmute      - unmute serial stream output")

def sendTelemetry():
    m = telemetryMode
    if m == TELEMETRY_RAW: printRawSensors()
    elif m == TELEMETRY_NORMALIZED: printNormalizedSensors()
    elif m == TELEMETRY_FILTERED: printFilteredSensors()
    elif m == TELEMETRY_CALIBRATION: printCalibration()
    elif m == TELEMETRY_PID: printPID()
    elif m == TELEMETRY_MOTOR: printMotors()
    elif m == TELEMETRY_TRACK: printTrackInformation()
    elif m == TELEMETRY_PERFORMANCE: printPerformance()
    elif m == TELEMETRY_ALL:
        printRobotState()
        printPID()
        printMotors()
        printTrackInformation()
        printLoopStatistics()

def processCommand(cmd):
    cmd = cmd.strip().lower()
    if not cmd: return

    if cmd == "raw": enableTelemetry(TELEMETRY_RAW)
    elif cmd == "norm": enableTelemetry(TELEMETRY_NORMALIZED)
    elif cmd == "filter": enableTelemetry(TELEMETRY_FILTERED)
    elif cmd == "calib": enableTelemetry(TELEMETRY_CALIBRATION)
    elif cmd == "pid": enableTelemetry(TELEMETRY_PID)
    elif cmd == "motors": enableTelemetry(TELEMETRY_MOTOR)
    elif cmd == "track": enableTelemetry(TELEMETRY_TRACK)
    elif cmd == "state": printRobotState()
    elif cmd == "loop": printLoopStatistics()
    elif cmd == "memory": printMemoryUsage()
    elif cmd == "battery": printBatteryStatus()
    elif cmd == "perf": printPerformance()
    elif cmd == "config": printConfig()
    elif cmd == "all": enableTelemetry(TELEMETRY_ALL)
    elif cmd in ("off", "stop"): disableTelemetry()
    elif cmd == "start": enableTelemetry(TELEMETRY_ALL)
    elif cmd == "clear":
        initializeTelemetry()
        print("Telemetry statistics cleared.")
    elif cmd == "status":
        print("Telemetry enabled: {}".format("YES" if telemetryEnabled else "NO"))
        print("Telemetry mode:    {}".format(telemetryMode))
        print("Stream muted:      {}".format("YES" if serialStreamMuted else "NO"))
    elif cmd == "mute": muteSerialStream(True)
    elif cmd == "unmute": muteSerialStream(False)
    elif cmd == "help": printHelp()
    else: print("Unknown command: '{}'. Type 'help' for available commands.".format(cmd))

def pollSerialCommands():
    global serialBuffer
    while select.select([sys.stdin], [], [], 0)[0]:
        ch = sys.stdin.read(1)
        if ch in ('\r', '\n'):
            if len(serialBuffer) > 0:
                processCommand(serialBuffer)
                serialBuffer = ""
        else:
            serialBuffer += ch

def updateTelemetry():
    global lastTelemetryPrintMs
    updateLoopTiming()
    pollSerialCommands()

    if not telemetryEnabled:
        return

    updateStatistics()

    now = time.ticks_ms()
    if time.ticks_diff(now, lastTelemetryPrintMs) >= TELEMETRY_PERIOD_MS:
        lastTelemetryPrintMs = now
        if sendTelemetrySerial():
            sendTelemetry()
            runDiagnostics()