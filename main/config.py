# =============================================================================
#  config.py
# -----------------------------------------------------------------------------
#  Central configuration file for the line follower robot (LFR) firmware.
#
#  This file is the single source of truth for every tunable constant,
#  shared global state variable, structure, enumeration, and cross-file
#  function prototype used across the project:
#
#      main.py      - setup()/loop() orchestration
#      config.py    - THIS FILE (configuration only)
#      sensors.py   - sensor acquisition, calibration, filtering
#      control.py   - PID, position estimation, track/junction detection
#      motor.py     - motor driver output, speed planning, recovery
#
#  This file intentionally contains NO sensor logic, NO PID logic, NO motor
#  driver logic, NO telemetry implementation, NO track detection logic, and
#  NO recovery logic. It defines only the constants, variables, structures,
#  enumerations, and function prototypes that those other files consume.
#
#  Coding constraints followed:
#      - No classes, no templates, no dynamic memory allocation.
#      - No .h/.cpp files - pure Arduino .ino sketch-tab style.
#      - constexpr used wherever a compile-time constant is appropriate;
#        const used for values that are logically constant but not
#        necessarily needed at compile time (e.g. structured defaults).
#      - Fixed-width integer types used everywhere instead of plain int
#        (uint8_t, uint16_t, uint32_t, int16_t, int32_t).
#      - Zero floating-point arithmetic allowed in this configuration file
#        and inside the high-speed loop paths of Sensors, Motor, Control,
#        or Telemetry.
# =============================================================================

# =============================================================================
#  1. FIRMWARE METADATA & BUILD IDENTIFIERS
# =============================================================================

ROBOT_NAME       = "LFR-PRO-16"
FIRMWARE_VERSION = "2.4.0"
BUILD_DATE       = "2025-02-15"
DEBUG_ENABLE     = True

# =============================================================================
#  2. HARDWARE PIN ASSIGNMENTS (RP2040 / Raspberry Pi Pico)
# =============================================================================

# --- CD74HC4067 16-Channel Analog Multiplexer --------------------------------
PIN_MUX_S0  = 0  # Multiplexer address line S0 (Bit 0)
PIN_MUX_S1  = 1  # Multiplexer address line S1 (Bit 1)
PIN_MUX_S2  = 2  # Multiplexer address line S2 (Bit 2)
PIN_MUX_S3  = 3  # Multiplexer address line S3 (Bit 3)
PIN_MUX_SIG = 26 # Multiplexer analog output connected to ADC0 (GPIO 26)

# --- TB6612FNG Dual H-Bridge Motor Driver -----------------------------------
PIN_MOTOR_AIN1 = 6  # Left motor direction input 1
PIN_MOTOR_AIN2 = 7  # Left motor direction input 2
PIN_MOTOR_PWMA = 8  # Left motor PWM speed control pin
PIN_MOTOR_BIN1 = 9  # Right motor direction input 1
PIN_MOTOR_BIN2 = 10 # Right motor direction input 2
PIN_MOTOR_PWMB = 11 # Right motor PWM speed control pin
PIN_MOTOR_STBY = 12 # Motor driver standby pin (HIGH = active, LOW = standby)

# --- User Interface & Status -------------------------------------------------
PIN_CALIBRATION_BUTTON = 13 # Push button for triggering sensor calibration
PIN_START_BUTTON       = 14 # Push button for enabling/disabling robot run
PIN_STATUS_LED         = 15 # WS2812B / NeoPixel status LED signal line
PIN_BATTERY_SENSE      = 27 # Optional battery voltage divider connected to ADC1 (GPIO 27)

# =============================================================================
#  3. SENSOR SUBSYSTEM CONFIGURATION
# =============================================================================

SENSOR_COUNT    = 16    # Total number of IR reflectance sensors in array
ADC_RESOLUTION  = 14    # ADC bit resolution
ADC_MAX         = 16383 # Maximum raw ADC value (2^14 - 1 = 16383)
NORMALIZED_MAX  = 1000  # Scaled sensor range (0 = white surface, 1000 = dark line)

# Multiplexer settling time in microseconds after changing channel bits
MUX_SETTLE_US   = 5

# Integer Exponential Moving Average (EMA) filter weight factors
# Combined weight divisor: EMA_OLD + EMA_NEW = 4 + 1 = 5
EMA_OLD         = 4
EMA_NEW         = 1

# Default initial minimum/maximum calibration bounds prior to calibration
DEFAULT_CALIB_MIN = 16383
DEFAULT_CALIB_MAX = 0

# Threshold for considering a sensor "active" (seeing the line)
SENSOR_ACTIVE_THRESHOLD = 300

# Sensor positions along array (mm relative to array center, index 0..15)
# Index 0 is leftmost sensor, Index 15 is rightmost sensor
SENSOR_POSITIONS_MM = [
    -52.5, -45.5, -38.5, -31.5, -24.5, -17.5, -10.5, -3.5,
      3.5,  10.5,  17.5,  24.5,  31.5,  38.5,  45.5, 52.5
]

# =============================================================================
#  4. CONTROL SUBSYSTEM & PID CONFIGURATION
# =============================================================================

# Loop timing constraints
CONTROL_LOOP_FREQ_HZ = 1000 # Target loop rate in Hertz
CONTROL_LOOP_DT_US   = 1000 # Microseconds per loop iteration (1000 us = 1 ms)

# Output scaling bounds
PID_MAX_OUTPUT       = 255  # Maximum absolute PID control effort
INTEGRAL_MAX_ACCUM   = 5000 # Maximum anti-windup clamp limit on accumulated integral

# Structure equivalent for PID Gains
class PIDGains:
    def __init__(self, Kp=0, Ki=0, Kd=0, Kp_num=0, Kp_den=1, Ki_num=0, Ki_den=1, Kd_num=0, Kd_den=1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kp_num = Kp_num
        self.Kp_den = Kp_den
        self.Ki_num = Ki_num
        self.Ki_den = Ki_den
        self.Kd_num = Kd_num
        self.Kd_den = Kd_den

# Fixed-point PID gain scaling representations (Numerator / Denominator)
# Avoids floating-point math during high-frequency loop execution

# Gains for straight track segments
GAIN_SET_STRAIGHT = PIDGains(
    Kp=2.5, Ki=0.001, Kd=18.0,
    Kp_num=5, Kp_den=2,
    Ki_num=1, Ki_den=1000,
    Kd_num=18, Kd_den=1
)

# Gains for shallow curves
GAIN_SET_CURVE = PIDGains(
    Kp=4.0, Ki=0.002, Kd=28.0,
    Kp_num=4, Kp_den=1,
    Ki_num=2, Ki_den=1000,
    Kd_num=28, Kd_den=1
)

# Gains for sharp curves and 90-degree turns
GAIN_SET_SHARP = PIDGains(
    Kp=6.5, Ki=0.005, Kd=45.0,
    Kp_num=13, Kp_den=2,
    Ki_num=5, Ki_den=1000,
    Kd_num=45, Kd_den=1
)

# Active threshold definitions for curve severity estimation
CURVE_SEVERITY_LOW    = 20
CURVE_SEVERITY_MEDIUM = 50
CURVE_SEVERITY_HIGH   = 80

# =============================================================================
#  5. MOTOR SUBSYSTEM & SPEED CONFIGURATION
# =============================================================================

MOTOR_DEAD_ZONE      = 35  # Minimum PWM required to overcome mechanical static friction
MOTOR_MAX_PWM        = 255 # Hard saturation ceiling for PWM hardware output
MOTOR_MIN_PWM        = 0   # Absolute minimum PWM output value
MOTOR_PWM_SATURATION = 255 # Alias for maximum PWM threshold limit

# Acceleration and deceleration slew rate limits (PWM change per loop cycle)
MOTOR_ACCEL_LIMIT    = 15  # Maximum allowable PWM increase per cycle
MOTOR_DECEL_LIMIT    = 25  # Maximum allowable PWM decrease per cycle

# Trim balance constants to equalize speed output between left and right motors
MOTOR_BALANCE_LEFT   = 0   # Additive offset applied to left motor PWM
MOTOR_BALANCE_RIGHT  = 0   # Additive offset applied to right motor PWM

# Base Speed Profile definitions
class SpeedProfile:
    def __init__(self, baseSpeed=0, maxSpeed=0, minSpeed=0, straightSpeed=0, curveSpeed=0, sharpCurveSpeed=0, turn90Speed=0):
        self.baseSpeed = baseSpeed
        self.maxSpeed = maxSpeed
        self.minSpeed = minSpeed
        self.straightSpeed = straightSpeed
        self.curveSpeed = curveSpeed
        self.sharpCurveSpeed = sharpCurveSpeed
        self.turn90Speed = turn90Speed

DEFAULT_SPEED_PROFILE = SpeedProfile(
    baseSpeed=180,
    maxSpeed=255,
    minSpeed=60,
    straightSpeed=220,
    curveSpeed=160,
    sharpCurveSpeed=110,
    turn90Speed=80
)

# =============================================================================
#  6. ENUMERATIONS & STATE DEFINITIONS
# =============================================================================

# Robot High-Level Operating Modes
MODE_BOOT          = 0
MODE_IDLE          = 1
MODE_CALIBRATION   = 2
MODE_READY         = 3
MODE_RUNNING       = 4
MODE_RECOVERY      = 5
MODE_ERROR         = 6
MODE_EMERGENCY_STOP= 7

# Track Segment Classifications
TRACK_STRAIGHT     = 0
TRACK_CURVE        = 1
TRACK_SHARP_CURVE  = 2
TRACK_90_TURN      = 3
TRACK_JUNCTION_T   = 4
TRACK_JUNCTION_CROSS=5
TRACK_GAP          = 6
TRACK_UNKNOWN      = 7

# Line Detection States
LINE_PRESENT       = 0
LINE_LOST_LEFT     = 1
LINE_LOST_RIGHT    = 2
LINE_FULL_BLANK    = 3

# =============================================================================
#  7. SYSTEM CONSTANTS & TIMEOUTS
# =============================================================================

CALIBRATION_TIME_MS      = 5000  # Total duration for auto-calibration sweep
LINE_LOST_TIMEOUT_MS     = 500   # Max allowed time off-line before entering recovery
RECOVERY_MAX_TIME_MS     = 1500  # Max duration allowed for active recovery maneuver
DEBOUNCE_DELAY_MS        = 50    # Button debounce period
STATUS_LED_BLINK_FAST_MS = 100   # LED period for error or fast alerts
STATUS_LED_BLINK_SLOW_MS = 500   # LED period for ready or idle state

# WS2812B Color Constants (RGB values 0..255)
COLOR_OFF_R = 0;   COLOR_OFF_G = 0;   COLOR_OFF_B = 0
COLOR_RED_R = 255; COLOR_RED_G = 0;   COLOR_RED_B = 0
COLOR_GRN_R = 0;   COLOR_GRN_G = 255; COLOR_GRN_B = 0
COLOR_BLU_R = 0;   COLOR_BLU_G = 0;   COLOR_BLU_B = 255
COLOR_YEL_R = 255; COLOR_YEL_G = 255; COLOR_YEL_B = 0
COLOR_MAG_R = 255; COLOR_MAG_G = 0;   COLOR_MAG_B = 255
COLOR_CYN_R = 0;   COLOR_CYN_G = 255; COLOR_CYN_B = 255
COLOR_WHITE_R=255; COLOR_WHITE_G=255; COLOR_WHITE_B=255

# =============================================================================
#  8. STRUCTURE & SNAPSHOT TYPE DEFINITIONS
# =============================================================================

class SensorData:
    def __init__(self, count=SENSOR_COUNT):
        self.raw        = [0] * count
        self.normalized = [0] * count
        self.filtered   = [0] * count
        self.minimum    = [DEFAULT_CALIB_MIN] * count
        self.maximum    = [DEFAULT_CALIB_MAX] * count

class MotorState:
    def __init__(self):
        self.speed     = 0
        self.pwm       = 0
        self.direction = 1 # 1 = Forward, -1 = Reverse, 0 = Stopped
        self.enabled   = False

class RobotDiagnostics:
    def __init__(self):
        self.loopTimeUs     = 0
        self.maxLoopTimeUs  = 0
        self.minLoopTimeUs  = 0xFFFFFFFF
        self.batteryVoltage = 0.0
        self.errorCount     = 0

# =============================================================================
#  9. SHARED GLOBAL STATE VARIABLES
# =============================================================================
# Primary system flags and mode tracking
robotMode           = MODE_BOOT
robotEnabled        = False
calibrationComplete = False
lineLost            = False

# Sensor metrics and calculated track position
linePosition        = 0 # Range -7500 to +7500 (Center is 0)
lineError           = 0 # Proportional error relative to central setpoint
lastLineError       = 0 # Line error value from previous loop iteration

# PID loop output metrics
integral            = 0 # Accumulated integral term
derivative          = 0 # Rate of error change
pidOutput           = 0 # Computed total motor differential effort

# Track classification variables
trackType           = TRACK_STRAIGHT
curveSeverity       = 0 # Calculated severity metric (0..100)

# Motor speed targets and active outputs
leftTargetPWM       = 0
rightTargetPWM      = 0
leftMotor           = 0 # Current applied PWM output to left motor driver
rightMotor          = 0 # Current applied PWM output to right motor driver

# Global data structures
sensor              = SensorData(SENSOR_COUNT)
leftMotorState      = MotorState()
rightMotorState     = MotorState()
diagnostics         = RobotDiagnostics()
activeSpeedProfile  = DEFAULT_SPEED_PROFILE
activeGains         = GAIN_SET_STRAIGHT