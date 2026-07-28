# =============================================================================
#  sensors.py
# -----------------------------------------------------------------------------
#  Sensor acquisition, calibration, normalization and filtering module for a
#  16-channel analog IR reflectance array line follower robot (LFR).
#
#  Hardware:
#      MCU              : RP2040 (Raspberry Pi Pico @ 125 MHz)
#      ADC Resolution    : 12-bit native scaled to 14-bit (0 .. 16383)
#      Multiplexer       : CD74HC4067 (16-channel analog mux, 4 address lines)
#      Sensors           : 16x Analog IR Reflectance Sensors
#
#  Responsibilities of this file (and ONLY this file):
#      - Multiplexer channel selection and analog acquisition
#      - Per-sensor calibration (min/max tracking)
#      - Per-sensor normalization to a fixed 0..1000 scale
#      - Per-sensor integer EMA (Exponential Moving Average) filtering
#      - Debug printing for MicroPython Serial
# =============================================================================

import machine
import time
import config

# Module Pin instances
pinMuxS0  = None
pinMuxS1  = None
pinMuxS2  = None
pinMuxS3  = None
adcSigaPin= None

# =============================================================================
#  initializeSensors()
# =============================================================================
def initializeSensors():
    global pinMuxS0, pinMuxS1, pinMuxS2, pinMuxS3, adcSigaPin

    pinMuxS0   = machine.Pin(config.PIN_MUX_S0, machine.Pin.OUT)
    pinMuxS1   = machine.Pin(config.PIN_MUX_S1, machine.Pin.OUT)
    pinMuxS2   = machine.Pin(config.PIN_MUX_S2, machine.Pin.OUT)
    pinMuxS3   = machine.Pin(config.PIN_MUX_S3, machine.Pin.OUT)
    adcSigaPin = machine.ADC(config.PIN_MUX_SIG)

    pinMuxS0.value(0)
    pinMuxS1.value(0)
    pinMuxS2.value(0)
    pinMuxS3.value(0)

    resetCalibration()

    for i in range(config.SENSOR_COUNT):
        config.sensor.raw[i]        = 0
        config.sensor.normalized[i] = 0
        config.sensor.filtered[i]   = 0

# =============================================================================
#  selectMuxChannel()
# =============================================================================
def selectMuxChannel(channel):
    pinMuxS0.value((channel >> 0) & 0x01)
    pinMuxS1.value((channel >> 1) & 0x01)
    pinMuxS2.value((channel >> 2) & 0x01)
    pinMuxS3.value((channel >> 3) & 0x01)
    time.sleep_us(config.MUX_SETTLE_US)

# =============================================================================
#  readRawSensor()
# =============================================================================
def readRawSensor(channel):
    selectMuxChannel(channel)
    # RP2040 read_u16 returns 0..65535, scale down to 14-bit ADC range (0..16383)
    raw16 = adcSigaPin.read_u16()
    return raw16 >> 2

# =============================================================================
#  resetCalibration()
# =============================================================================
def resetCalibration():
    for i in range(config.SENSOR_COUNT):
        config.sensor.minimum[i] = config.DEFAULT_CALIB_MIN
        config.sensor.maximum[i] = config.DEFAULT_CALIB_MAX

# =============================================================================
#  calibrateSensors()
# =============================================================================
def calibrateSensors():
    for i in range(config.SENSOR_COUNT):
        rawVal = readRawSensor(i)
        config.sensor.raw[i] = rawVal

        if rawVal < config.sensor.minimum[i]:
            config.sensor.minimum[i] = rawVal
        if rawVal > config.sensor.maximum[i]:
            config.sensor.maximum[i] = rawVal

# =============================================================================
#  readSensors()
# =============================================================================
def readSensors():
    for i in range(config.SENSOR_COUNT):
        config.sensor.raw[i] = readRawSensor(i)

# =============================================================================
#  normalizeSensors()
# =============================================================================
def normalizeSensors():
    for i in range(config.SENSOR_COUNT):
        rawVal = config.sensor.raw[i]
        minVal = config.sensor.minimum[i]
        maxVal = config.sensor.maximum[i]

        if maxVal <= minVal:
            config.sensor.normalized[i] = 0
            continue

        if rawVal <= minVal:
            norm = 0
        elif rawVal >= maxVal:
            norm = config.NORMALIZED_MAX
        else:
            span = maxVal - minVal
            norm = ((rawVal - minVal) * config.NORMALIZED_MAX) // span

        config.sensor.normalized[i] = norm

# =============================================================================
#  filterSensors()
# =============================================================================
def filterSensors():
    for i in range(config.SENSOR_COUNT):
        normVal = config.sensor.normalized[i]
        prevFilt = config.sensor.filtered[i]

        filt = (prevFilt * config.EMA_OLD + normVal * config.EMA_NEW) // (config.EMA_OLD + config.EMA_NEW)
        config.sensor.filtered[i] = filt

# =============================================================================
#  updateSensors()
# =============================================================================
def updateSensors():
    readSensors()
    normalizeSensors()
    filterSensors()

# =============================================================================
#  getSensor() / isSensorActive()
# =============================================================================
def getSensor(index):
    return config.sensor.filtered[index]

def isSensorActive(index, threshold):
    return config.sensor.filtered[index] > threshold