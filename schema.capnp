@0xac6c6e68f4d0c7e2;


#durin has 2Mbaud UART port and a TCP socket on port 1337

const streamPeriodMax :UInt16 = 65535;
const streamPeriodMin :UInt16 = 0;
const durinTcpPort :UInt16 = 1337;
const durinBaud :UInt32 = 2000000;
const lengthMask :UInt16 = 0x0fff;
const metaMask :UInt16 = 0xf000;

## all types are implemented here so this is the message you want to listen for. Then you can check which field of the union you actually got
struct DurinBase {
    union {
        reject @0 :Reject;
        acknowledge @1 :Acknowledge;
        ping @31 :Ping;
        powerOff @2 :PowerOff;
        setRobotVelocity @3 :SetRobotVelocity;
        setWheelVelocity @4 :SetWheelVelocity;
        setBuzzer @5 :SetBuzzer;
        setLed @6 :SetLed;

        enableStreaming @7 :EnableStreaming;
        disableStreaming @8 :DisableStreaming;

        setTofStreamPeriod @9 :SetTofStreamPeriod;
        getTofObservations @10 :GetTofObservations;
        setTofResolution @11 :SetTofResolution;
        tofObservations @12 :TofObservations;
        enableTofStatus @35 :EnableTofStatus;

        setImuStreamPeriod @13 :SetImuStreamPeriod;
        getImuMeasurement @14 :GetImuMeasurement;
        imuMeasurement @15 :ImuMeasurement;

        setSystemStatusStreamPeriod @16 :SetSystemStatusStreamPeriod;
        getSystemStatus @17 :GetSystemStatus;
        systemStatus @18 :SystemStatus;

        setPositionStreamPeriod @21 :SetPositionStreamPeriod;
        getPosition @22 :GetPosition;
        position @23 :Position;

        setUwbStreamPeriod @32 :SetUwbStreamPeriod;
        getUwbNodes @19 :GetUwbNodes;
        uwbNodes @20 :UwbNodes;

        setWifiConfig @24 :SetWifiConfig;
        setNodeId @25 :SetNodeId;
        textLogging @26 :TextLogging;
        otaUpdateCommit @27 :OtaUpdateCommit;
        otaUpdate @28 :OtaUpdate;
        enableLogging @29 :EnableLogging;
        otaUpdateBegin @30 :OtaUpdateBegin;

        getSystemInfo @33 :GetSystemInfo;
        systemInfo @34 :SystemInfo;
    }
}

# Response signifying that the previous message was invalid or ignored
struct Reject {

}

# Response signifying that the message was processed succesfully
struct Acknowledge {

}


### control

# powers off durin
struct PowerOff {

}

# Sets the velocity of durin, positive Y is forward, rotation is counterclockwise
# velocity in millimeters/second
# rotaional velocity in degrees/s
struct SetRobotVelocity {
    velocityXMms @0 :Int16;
    velocityYMms @1 :Int16;
    rotationDegs @2 :Int16;
}

# Sets the the velocity of each individual wheel, positive is forward
# velocity in millimeters/second
struct SetWheelVelocity {
    wheelFrontLeftMms @0 :Int16;
    wheelFrontRightMms @1 :Int16;
    wheelBackLeftMms @2 :Int16;
    wheelBackRightMms @3 :Int16;
}

# enables/disables the onboard buzzer
struct SetBuzzer {
    enabled @0 :Bool;
}

# sets the rgb-led;
# the value for each LED is scaled between 0 and 255
struct SetLed {
    ledR @0 :UInt8;
    ledG @1 :UInt8;
    ledB @2 :UInt8;
}

# Durin can stream telemetry over UDP and/or UART. This command sets the IP and Port for the receiving server and starts streaming
# UUART Baudrate is locked at 2Mbaud
struct EnableStreaming {
    destination :union {
        uartOnly @0 :Void;
        udpOnly :group {
            ip @1 :List(UInt8);
            port @2 :UInt16;
        }
        uartAndUdp :group {
            ip @3 :List(UInt8);
            port @4 :UInt16;
        }
    }
}


# Disables the streaming of telemetry data
struct DisableStreaming {

}


### TOF measurements

# Sets the rate at which data will be streamed. The period is in MS
# Set to STREAM_PERIOD_MAX (65535) to not stream any data
# Set to STREAM_PERIOD_MIN (0) to only stream new values at the rate they are sampled by durin
# any other value will send the data at that period even if means sending stale data
struct SetTofStreamPeriod {
    periodMs @0 :UInt16;
}

# Polls a single ToF observation, responds over the same channel as the request was sent (UART or TCP)
struct GetTofObservations {
    ids @0 :List(UInt8);
}

# Sets the resolution and update rate for the TOF sensors
enum TofResolutions {
    resolution4x4rate60Hz @0;
    resolution8x8rate15Hz @1;
}
struct SetTofResolution {
    resolution @0 :TofResolutions;
}

struct EnableTofStatus {
    enabled @0 :Bool;
}

# A list with TOF observations
# Durin has 8 ToF sensors each spanning 45° degrees.
# The ID tells the angle ccw from the front of according to the formula id*45°. So id 0 faces forward, 2 left and 6 right
# The ranging measurements come in a list of distances in row-major order
# bits 0:14 contain the distance in mm
# bits 15:16 contain an enum specifying the state
## 0 - valid
## 1 - 50% valid
## 2 - invalid
## 3 - not updated
struct TofObservations {
    observations @0 :List(TofObservation);
    struct TofObservation {
        id @0 :UInt8;
        ranges @1 :List(UInt16);
    }
}

# Sets the rate at which data will be streamed. The period is in MS
# Set to STREAM_PERIOD_MAX (65535) to not stream any data
# Set to STREAM_PERIOD_MIN (0) to only stream new values at the rate they are sampled by durin
# any other value will send the data at that period even if means sending stale data
struct SetImuStreamPeriod {
    periodMs @0 :UInt16;
}

# Polls a single IMU measurement, responds over the same channel as the request was sent (UART or TCP)
struct GetImuMeasurement {

}

# an imu measurement
# raw values. Use the scaling factors to get metric
const accelerometerToGs :Float32 = 1;
const gyroscopeToRadiansPerSecond :Float32 = 1;
const magnetometerToMicroTesla :Float32 = 1;
struct ImuMeasurement {
    accelerometerX @0 :Int16;
    accelerometerY @1 :Int16;
    accelerometerZ @2 :Int16;
    gyroscopeX @3 :Int16;
    gyroscopeY @4 :Int16;
    gyroscopeZ @5 :Int16;
    magnetometerX @6 :Int16;
    magnetometerY @7 :Int16;
    magnetometerZ @8 :Int16;
}


### system status

# Sets the rate at which data will be streamed. The period is in MS
# Set to STREAM_PERIOD_MAX (65535) to not stream any data
# Set to STREAM_PERIOD_MIN (0) to only stream new values at the rate they are sampled by durin
# any other value will send the data at that period even if means sending stale data
struct SetSystemStatusStreamPeriod {
    periodMs @0 :UInt16;
}

# Polls the system status, responds over the same channel as the request was sent (UART or TCP)
struct GetSystemStatus {

}

# batteryvoltage in millivolts
struct SystemStatus {
    batteryMv @0 :UInt16;
    batteryPercent @1 :UInt8;
    batteryDischarge @2 :UInt8;
}

### uwb ranging

# polls one node over UWB
struct GetDistanceMeasurement {
    nodeId @0 :UInt8;
}

struct DistanceMeasurement {
    nodeId @0 :UInt8;
    distanceMm @1 :UInt32;
    position :union {
        unknown @2 :Void;
        vector :group {
            x @3 :Int32;
            y @4 :Int32;
            z @5 :Int32;
        }
    }
}

# Sets the rate at which data will be streamed. The period is in MS
# Set to STREAM_PERIOD_MAX (65535) to not stream any data
# Set to STREAM_PERIOD_MIN (0) to only stream new values at the rate they are sampled by durin
# any other value will send the data at that period even if means sending stale data
struct SetPositionStreamPeriod {
    periodMs @0 :UInt16;
}

# gets the position as caluclated from UWB beacons, responds over the same channel as the request was sent (UART or TCP)
struct GetPosition {

}

enum UwbNodePurpose {
    origin @0;
    x @1;
    y @2;
    z @3;
    repeater @4;
    passive @5;
    user @6;
}

struct UwbNode {
    nodeId @0 :UInt8;
    purpose @1 :UwbNodePurpose;
    distanceMm @2 :UInt32;
    flags @3 :UInt32;
    position :union {
        unknown @4 :Void;
        vectorMm :group {
            x @5 :Int32;
            y @6 :Int32;
            z @7 :Int32;
        }
    }
}

struct GetUwbNodes {

}

struct SetUwbStreamPeriod {
    periodMs @0 :UInt16;
}

struct UwbNodes {
    nodes @0 :List(UwbNode);
}

# X, Y, Z in millimeters
struct Position {
    union {
        unknown @0 :Void;
        vectorMm :group {
            x @1 :Int32;
            y @2 :Int32;
            z @3 :Int32;
        }
    }
}


### system configuration
struct SetWifiConfig {
    ssid @0 :Text;
    password @1 :Text;
    index @2 :UInt8;
}

struct SetNodeId {
    nodeId @0 :UInt8;
}


# generic way to ouput a string for debugging
struct TextLogging {
    log @0 :Text;
}

# for runtime firmware updates
struct OtaUpdateBegin {

}

struct OtaUpdateCommit {

}

struct OtaUpdate {
    data @0 :Data;
}

# enables console output through TextLogging messages
struct EnableLogging {
    union {
        disabled @0 :Void;
        tcp @1 :Void;
        uart @2 :Void;
        both @3 :Void;
    }
}

struct Ping {

}

enum ErrorType {
    wheelVelocityTooHigh @0;
    positioningHighDisplacement @1;
}

struct Error {
    type @0 :ErrorType;
    string @1 :Text;
}


struct GetSystemInfo {

}

struct SystemInfo {
    mac @0 :Text;
    ip @1 :Text;
    hostname @2 :Text;
    id @3 :UInt8;
    uptimeMs @4 :UInt32;
}