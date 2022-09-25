#ifndef CAPN_AC6C6E68F4D0C7E2
#define CAPN_AC6C6E68F4D0C7E2
/* AUTO GENERATED - DO NOT EDIT */
#include <capnp_c.h>

#if CAPN_VERSION != 1
#error "version mismatch between capnp_c.h and generated code"
#endif

#ifndef capnp_nowarn
# ifdef __GNUC__
#  define capnp_nowarn __extension__
# else
#  define capnp_nowarn
# endif
#endif


#ifdef __cplusplus
extern "C" {
#endif

struct DurinBase;
struct Reject;
struct Acknowledge;
struct PowerOff;
struct SetRobotVelocity;
struct SetWheelVelocity;
struct SetBuzzer;
struct SetLed;
struct EnableStreaming;
struct DisableStreaming;
struct SetTofStreamPeriod;
struct GetTofObservations;
struct SetTofResolution;
struct TofObservations;
struct TofObservations_TofObservation;
struct SetImuStreamPeriod;
struct GetImuMeasurement;
struct ImuMeasurement;
struct SetSystemStatusStreamPeriod;
struct GetSystemStatus;
struct SystemStatus;
struct GetDistanceMeasurement;
struct DistanceMeasurement;
struct SetPositionStreamPeriod;
struct GetPosition;
struct Position;
struct SetWifiConfig;
struct SetNodeId;
struct TextLogging;
struct OtaUpdateBegin;
struct OtaUpdateCommit;
struct OtaUpdate;
struct EnableLogging;

typedef struct {capn_ptr p;} DurinBase_ptr;
typedef struct {capn_ptr p;} Reject_ptr;
typedef struct {capn_ptr p;} Acknowledge_ptr;
typedef struct {capn_ptr p;} PowerOff_ptr;
typedef struct {capn_ptr p;} SetRobotVelocity_ptr;
typedef struct {capn_ptr p;} SetWheelVelocity_ptr;
typedef struct {capn_ptr p;} SetBuzzer_ptr;
typedef struct {capn_ptr p;} SetLed_ptr;
typedef struct {capn_ptr p;} EnableStreaming_ptr;
typedef struct {capn_ptr p;} DisableStreaming_ptr;
typedef struct {capn_ptr p;} SetTofStreamPeriod_ptr;
typedef struct {capn_ptr p;} GetTofObservations_ptr;
typedef struct {capn_ptr p;} SetTofResolution_ptr;
typedef struct {capn_ptr p;} TofObservations_ptr;
typedef struct {capn_ptr p;} TofObservations_TofObservation_ptr;
typedef struct {capn_ptr p;} SetImuStreamPeriod_ptr;
typedef struct {capn_ptr p;} GetImuMeasurement_ptr;
typedef struct {capn_ptr p;} ImuMeasurement_ptr;
typedef struct {capn_ptr p;} SetSystemStatusStreamPeriod_ptr;
typedef struct {capn_ptr p;} GetSystemStatus_ptr;
typedef struct {capn_ptr p;} SystemStatus_ptr;
typedef struct {capn_ptr p;} GetDistanceMeasurement_ptr;
typedef struct {capn_ptr p;} DistanceMeasurement_ptr;
typedef struct {capn_ptr p;} SetPositionStreamPeriod_ptr;
typedef struct {capn_ptr p;} GetPosition_ptr;
typedef struct {capn_ptr p;} Position_ptr;
typedef struct {capn_ptr p;} SetWifiConfig_ptr;
typedef struct {capn_ptr p;} SetNodeId_ptr;
typedef struct {capn_ptr p;} TextLogging_ptr;
typedef struct {capn_ptr p;} OtaUpdateBegin_ptr;
typedef struct {capn_ptr p;} OtaUpdateCommit_ptr;
typedef struct {capn_ptr p;} OtaUpdate_ptr;
typedef struct {capn_ptr p;} EnableLogging_ptr;

typedef struct {capn_ptr p;} DurinBase_list;
typedef struct {capn_ptr p;} Reject_list;
typedef struct {capn_ptr p;} Acknowledge_list;
typedef struct {capn_ptr p;} PowerOff_list;
typedef struct {capn_ptr p;} SetRobotVelocity_list;
typedef struct {capn_ptr p;} SetWheelVelocity_list;
typedef struct {capn_ptr p;} SetBuzzer_list;
typedef struct {capn_ptr p;} SetLed_list;
typedef struct {capn_ptr p;} EnableStreaming_list;
typedef struct {capn_ptr p;} DisableStreaming_list;
typedef struct {capn_ptr p;} SetTofStreamPeriod_list;
typedef struct {capn_ptr p;} GetTofObservations_list;
typedef struct {capn_ptr p;} SetTofResolution_list;
typedef struct {capn_ptr p;} TofObservations_list;
typedef struct {capn_ptr p;} TofObservations_TofObservation_list;
typedef struct {capn_ptr p;} SetImuStreamPeriod_list;
typedef struct {capn_ptr p;} GetImuMeasurement_list;
typedef struct {capn_ptr p;} ImuMeasurement_list;
typedef struct {capn_ptr p;} SetSystemStatusStreamPeriod_list;
typedef struct {capn_ptr p;} GetSystemStatus_list;
typedef struct {capn_ptr p;} SystemStatus_list;
typedef struct {capn_ptr p;} GetDistanceMeasurement_list;
typedef struct {capn_ptr p;} DistanceMeasurement_list;
typedef struct {capn_ptr p;} SetPositionStreamPeriod_list;
typedef struct {capn_ptr p;} GetPosition_list;
typedef struct {capn_ptr p;} Position_list;
typedef struct {capn_ptr p;} SetWifiConfig_list;
typedef struct {capn_ptr p;} SetNodeId_list;
typedef struct {capn_ptr p;} TextLogging_list;
typedef struct {capn_ptr p;} OtaUpdateBegin_list;
typedef struct {capn_ptr p;} OtaUpdateCommit_list;
typedef struct {capn_ptr p;} OtaUpdate_list;
typedef struct {capn_ptr p;} EnableLogging_list;

enum SetTofResolution_TofResolutions {
	SetTofResolution_TofResolutions_resolution4x4rate30Hz = 0,
	SetTofResolution_TofResolutions_resolution8x8rate15Hz = 1
};
extern uint16_t streamPeriodMax;
extern uint16_t streamPeriodMin;
extern uint16_t durinTcpPort;
enum DurinBase_message_which {
	DurinBase_message_reject = 0,
	DurinBase_message_acknowledge = 1,
	DurinBase_message_powerOff = 2,
	DurinBase_message_setRobotVelocity = 3,
	DurinBase_message_setWheelVelocity = 4,
	DurinBase_message_setBuzzer = 5,
	DurinBase_message_setLed = 6,
	DurinBase_message_enableStreaming = 7,
	DurinBase_message_disableStreaming = 8,
	DurinBase_message_setTofStreamPeriod = 9,
	DurinBase_message_getTofObservations = 10,
	DurinBase_message_setTofResolution = 11,
	DurinBase_message_tofObservations = 12,
	DurinBase_message_setImuStreamPeriod = 13,
	DurinBase_message_getImuMeasurement = 14,
	DurinBase_message_imuMeasurement = 15,
	DurinBase_message_setSystemStatusStreamPeriod = 16,
	DurinBase_message_getSystemStatus = 17,
	DurinBase_message_systemStatus = 18,
	DurinBase_message_getDistanceMeasurement = 19,
	DurinBase_message_distanceMeasurement = 20,
	DurinBase_message_setPositionStreamPeriod = 21,
	DurinBase_message_getPosition = 22,
	DurinBase_message_position = 23,
	DurinBase_message_setWifiConfig = 24,
	DurinBase_message_setNodeId = 25,
	DurinBase_message_textLogging = 26,
	DurinBase_message_otaUpdateCommit = 27,
	DurinBase_message_otaUpdate = 28,
	DurinBase_message_enableLogging = 29,
	DurinBase_message_otaUpdateBegin = 30
};

struct DurinBase {
	enum DurinBase_message_which message_which;
	capnp_nowarn union {
		Reject_ptr reject;
		Acknowledge_ptr acknowledge;
		PowerOff_ptr powerOff;
		SetRobotVelocity_ptr setRobotVelocity;
		SetWheelVelocity_ptr setWheelVelocity;
		SetBuzzer_ptr setBuzzer;
		SetLed_ptr setLed;
		EnableStreaming_ptr enableStreaming;
		DisableStreaming_ptr disableStreaming;
		SetTofStreamPeriod_ptr setTofStreamPeriod;
		GetTofObservations_ptr getTofObservations;
		SetTofResolution_ptr setTofResolution;
		TofObservations_ptr tofObservations;
		SetImuStreamPeriod_ptr setImuStreamPeriod;
		GetImuMeasurement_ptr getImuMeasurement;
		ImuMeasurement_ptr imuMeasurement;
		SetSystemStatusStreamPeriod_ptr setSystemStatusStreamPeriod;
		GetSystemStatus_ptr getSystemStatus;
		SystemStatus_ptr systemStatus;
		GetDistanceMeasurement_ptr getDistanceMeasurement;
		DistanceMeasurement_ptr distanceMeasurement;
		SetPositionStreamPeriod_ptr setPositionStreamPeriod;
		GetPosition_ptr getPosition;
		Position_ptr position;
		SetWifiConfig_ptr setWifiConfig;
		SetNodeId_ptr setNodeId;
		TextLogging_ptr textLogging;
		OtaUpdateCommit_ptr otaUpdateCommit;
		OtaUpdate_ptr otaUpdate;
		EnableLogging_ptr enableLogging;
		OtaUpdateBegin_ptr otaUpdateBegin;
	} message;
};

static const size_t DurinBase_word_count = 1;

static const size_t DurinBase_pointer_count = 1;

static const size_t DurinBase_struct_bytes_count = 16;


capnp_nowarn struct Reject {
};

static const size_t Reject_word_count = 0;

static const size_t Reject_pointer_count = 0;

static const size_t Reject_struct_bytes_count = 0;


capnp_nowarn struct Acknowledge {
};

static const size_t Acknowledge_word_count = 0;

static const size_t Acknowledge_pointer_count = 0;

static const size_t Acknowledge_struct_bytes_count = 0;


capnp_nowarn struct PowerOff {
};

static const size_t PowerOff_word_count = 0;

static const size_t PowerOff_pointer_count = 0;

static const size_t PowerOff_struct_bytes_count = 0;


struct SetRobotVelocity {
	int16_t velocityXMms;
	int16_t velocityYMms;
	int16_t rotationDegs;
};

static const size_t SetRobotVelocity_word_count = 1;

static const size_t SetRobotVelocity_pointer_count = 0;

static const size_t SetRobotVelocity_struct_bytes_count = 8;


struct SetWheelVelocity {
	int16_t wheelFrontLeftMms;
	int16_t wheelFrontRightMms;
	int16_t wheelBackLeftMms;
	int16_t wheelBackRightMms;
};

static const size_t SetWheelVelocity_word_count = 1;

static const size_t SetWheelVelocity_pointer_count = 0;

static const size_t SetWheelVelocity_struct_bytes_count = 8;


struct SetBuzzer {
	unsigned enabled : 1;
};

static const size_t SetBuzzer_word_count = 1;

static const size_t SetBuzzer_pointer_count = 0;

static const size_t SetBuzzer_struct_bytes_count = 8;


struct SetLed {
	uint8_t ledR;
	uint8_t ledG;
	uint8_t ledB;
};

static const size_t SetLed_word_count = 1;

static const size_t SetLed_pointer_count = 0;

static const size_t SetLed_struct_bytes_count = 8;

enum EnableStreaming_destination_which {
	EnableStreaming_destination_uartOnly = 0,
	EnableStreaming_destination_udpOnly = 1,
	EnableStreaming_destination_uartAndUdp = 2
};

struct EnableStreaming {
	enum EnableStreaming_destination_which destination_which;
	capnp_nowarn union {
		capnp_nowarn struct {
			capn_list8 ip;
			uint16_t port;
		} udpOnly;
		capnp_nowarn struct {
			capn_list8 ip;
			uint16_t port;
		} uartAndUdp;
	} destination;
};

static const size_t EnableStreaming_word_count = 1;

static const size_t EnableStreaming_pointer_count = 1;

static const size_t EnableStreaming_struct_bytes_count = 16;


capnp_nowarn struct DisableStreaming {
};

static const size_t DisableStreaming_word_count = 0;

static const size_t DisableStreaming_pointer_count = 0;

static const size_t DisableStreaming_struct_bytes_count = 0;


struct SetTofStreamPeriod {
	uint16_t periodMs;
};

static const size_t SetTofStreamPeriod_word_count = 1;

static const size_t SetTofStreamPeriod_pointer_count = 0;

static const size_t SetTofStreamPeriod_struct_bytes_count = 8;


struct GetTofObservations {
	capn_list8 ids;
};

static const size_t GetTofObservations_word_count = 0;

static const size_t GetTofObservations_pointer_count = 1;

static const size_t GetTofObservations_struct_bytes_count = 8;


struct SetTofResolution {
	enum SetTofResolution_TofResolutions resolution;
};

static const size_t SetTofResolution_word_count = 1;

static const size_t SetTofResolution_pointer_count = 0;

static const size_t SetTofResolution_struct_bytes_count = 8;


struct TofObservations {
	TofObservations_TofObservation_list observations;
};

static const size_t TofObservations_word_count = 0;

static const size_t TofObservations_pointer_count = 1;

static const size_t TofObservations_struct_bytes_count = 8;


struct TofObservations_TofObservation {
	uint8_t id;
	capn_list16 ranges;
};

static const size_t TofObservations_TofObservation_word_count = 1;

static const size_t TofObservations_TofObservation_pointer_count = 1;

static const size_t TofObservations_TofObservation_struct_bytes_count = 16;


struct SetImuStreamPeriod {
	uint16_t periodMs;
};

static const size_t SetImuStreamPeriod_word_count = 1;

static const size_t SetImuStreamPeriod_pointer_count = 0;

static const size_t SetImuStreamPeriod_struct_bytes_count = 8;


capnp_nowarn struct GetImuMeasurement {
};

static const size_t GetImuMeasurement_word_count = 0;

static const size_t GetImuMeasurement_pointer_count = 0;

static const size_t GetImuMeasurement_struct_bytes_count = 0;


struct ImuMeasurement {
	float accelerometerXG;
	float accelerometerYG;
	float accelerometerZG;
	float gyroscopeXRads;
	float gyroscopeYRads;
	float gyroscopeZRads;
	float magnetometerXUt;
	float magnetometerYUt;
	float magnetometerZUt;
};

static const size_t ImuMeasurement_word_count = 5;

static const size_t ImuMeasurement_pointer_count = 0;

static const size_t ImuMeasurement_struct_bytes_count = 40;


struct SetSystemStatusStreamPeriod {
	uint16_t periodMs;
};

static const size_t SetSystemStatusStreamPeriod_word_count = 1;

static const size_t SetSystemStatusStreamPeriod_pointer_count = 0;

static const size_t SetSystemStatusStreamPeriod_struct_bytes_count = 8;


capnp_nowarn struct GetSystemStatus {
};

static const size_t GetSystemStatus_word_count = 0;

static const size_t GetSystemStatus_pointer_count = 0;

static const size_t GetSystemStatus_struct_bytes_count = 0;


struct SystemStatus {
	uint16_t batteryMv;
	uint8_t batteryPercent;
};

static const size_t SystemStatus_word_count = 1;

static const size_t SystemStatus_pointer_count = 0;

static const size_t SystemStatus_struct_bytes_count = 8;


struct GetDistanceMeasurement {
	uint8_t nodeId;
};

static const size_t GetDistanceMeasurement_word_count = 1;

static const size_t GetDistanceMeasurement_pointer_count = 0;

static const size_t GetDistanceMeasurement_struct_bytes_count = 8;

enum DistanceMeasurement_position_which {
	DistanceMeasurement_position_unknown = 0,
	DistanceMeasurement_position_vector = 1
};

struct DistanceMeasurement {
	uint8_t nodeId;
	uint32_t distanceMm;
	enum DistanceMeasurement_position_which position_which;
	capnp_nowarn union {
		capnp_nowarn struct {
			int32_t x;
			int32_t y;
			int32_t z;
		} vector;
	} position;
};

static const size_t DistanceMeasurement_word_count = 3;

static const size_t DistanceMeasurement_pointer_count = 0;

static const size_t DistanceMeasurement_struct_bytes_count = 24;


struct SetPositionStreamPeriod {
	uint16_t periodMs;
};

static const size_t SetPositionStreamPeriod_word_count = 1;

static const size_t SetPositionStreamPeriod_pointer_count = 0;

static const size_t SetPositionStreamPeriod_struct_bytes_count = 8;


capnp_nowarn struct GetPosition {
};

static const size_t GetPosition_word_count = 0;

static const size_t GetPosition_pointer_count = 0;

static const size_t GetPosition_struct_bytes_count = 0;

enum Position_position_which {
	Position_position_unknown = 0,
	Position_position_vector = 1
};

struct Position {
	enum Position_position_which position_which;
	capnp_nowarn union {
		capnp_nowarn struct {
			int32_t x;
			int32_t y;
			int32_t z;
		} vector;
	} position;
};

static const size_t Position_word_count = 2;

static const size_t Position_pointer_count = 0;

static const size_t Position_struct_bytes_count = 16;


struct SetWifiConfig {
	capn_text ssid;
	capn_text password;
};

static const size_t SetWifiConfig_word_count = 0;

static const size_t SetWifiConfig_pointer_count = 2;

static const size_t SetWifiConfig_struct_bytes_count = 16;


struct SetNodeId {
	uint8_t nodeId;
};

static const size_t SetNodeId_word_count = 1;

static const size_t SetNodeId_pointer_count = 0;

static const size_t SetNodeId_struct_bytes_count = 8;


struct TextLogging {
	capn_text log;
};

static const size_t TextLogging_word_count = 0;

static const size_t TextLogging_pointer_count = 1;

static const size_t TextLogging_struct_bytes_count = 8;


capnp_nowarn struct OtaUpdateBegin {
};

static const size_t OtaUpdateBegin_word_count = 0;

static const size_t OtaUpdateBegin_pointer_count = 0;

static const size_t OtaUpdateBegin_struct_bytes_count = 0;


capnp_nowarn struct OtaUpdateCommit {
};

static const size_t OtaUpdateCommit_word_count = 0;

static const size_t OtaUpdateCommit_pointer_count = 0;

static const size_t OtaUpdateCommit_struct_bytes_count = 0;


struct OtaUpdate {
	capn_data data;
};

static const size_t OtaUpdate_word_count = 0;

static const size_t OtaUpdate_pointer_count = 1;

static const size_t OtaUpdate_struct_bytes_count = 8;


struct EnableLogging {
	unsigned enabled : 1;
};

static const size_t EnableLogging_word_count = 1;

static const size_t EnableLogging_pointer_count = 0;

static const size_t EnableLogging_struct_bytes_count = 8;


DurinBase_ptr new_DurinBase(struct capn_segment*);
Reject_ptr new_Reject(struct capn_segment*);
Acknowledge_ptr new_Acknowledge(struct capn_segment*);
PowerOff_ptr new_PowerOff(struct capn_segment*);
SetRobotVelocity_ptr new_SetRobotVelocity(struct capn_segment*);
SetWheelVelocity_ptr new_SetWheelVelocity(struct capn_segment*);
SetBuzzer_ptr new_SetBuzzer(struct capn_segment*);
SetLed_ptr new_SetLed(struct capn_segment*);
EnableStreaming_ptr new_EnableStreaming(struct capn_segment*);
DisableStreaming_ptr new_DisableStreaming(struct capn_segment*);
SetTofStreamPeriod_ptr new_SetTofStreamPeriod(struct capn_segment*);
GetTofObservations_ptr new_GetTofObservations(struct capn_segment*);
SetTofResolution_ptr new_SetTofResolution(struct capn_segment*);
TofObservations_ptr new_TofObservations(struct capn_segment*);
TofObservations_TofObservation_ptr new_TofObservations_TofObservation(struct capn_segment*);
SetImuStreamPeriod_ptr new_SetImuStreamPeriod(struct capn_segment*);
GetImuMeasurement_ptr new_GetImuMeasurement(struct capn_segment*);
ImuMeasurement_ptr new_ImuMeasurement(struct capn_segment*);
SetSystemStatusStreamPeriod_ptr new_SetSystemStatusStreamPeriod(struct capn_segment*);
GetSystemStatus_ptr new_GetSystemStatus(struct capn_segment*);
SystemStatus_ptr new_SystemStatus(struct capn_segment*);
GetDistanceMeasurement_ptr new_GetDistanceMeasurement(struct capn_segment*);
DistanceMeasurement_ptr new_DistanceMeasurement(struct capn_segment*);
SetPositionStreamPeriod_ptr new_SetPositionStreamPeriod(struct capn_segment*);
GetPosition_ptr new_GetPosition(struct capn_segment*);
Position_ptr new_Position(struct capn_segment*);
SetWifiConfig_ptr new_SetWifiConfig(struct capn_segment*);
SetNodeId_ptr new_SetNodeId(struct capn_segment*);
TextLogging_ptr new_TextLogging(struct capn_segment*);
OtaUpdateBegin_ptr new_OtaUpdateBegin(struct capn_segment*);
OtaUpdateCommit_ptr new_OtaUpdateCommit(struct capn_segment*);
OtaUpdate_ptr new_OtaUpdate(struct capn_segment*);
EnableLogging_ptr new_EnableLogging(struct capn_segment*);

DurinBase_list new_DurinBase_list(struct capn_segment*, int len);
Reject_list new_Reject_list(struct capn_segment*, int len);
Acknowledge_list new_Acknowledge_list(struct capn_segment*, int len);
PowerOff_list new_PowerOff_list(struct capn_segment*, int len);
SetRobotVelocity_list new_SetRobotVelocity_list(struct capn_segment*, int len);
SetWheelVelocity_list new_SetWheelVelocity_list(struct capn_segment*, int len);
SetBuzzer_list new_SetBuzzer_list(struct capn_segment*, int len);
SetLed_list new_SetLed_list(struct capn_segment*, int len);
EnableStreaming_list new_EnableStreaming_list(struct capn_segment*, int len);
DisableStreaming_list new_DisableStreaming_list(struct capn_segment*, int len);
SetTofStreamPeriod_list new_SetTofStreamPeriod_list(struct capn_segment*, int len);
GetTofObservations_list new_GetTofObservations_list(struct capn_segment*, int len);
SetTofResolution_list new_SetTofResolution_list(struct capn_segment*, int len);
TofObservations_list new_TofObservations_list(struct capn_segment*, int len);
TofObservations_TofObservation_list new_TofObservations_TofObservation_list(struct capn_segment*, int len);
SetImuStreamPeriod_list new_SetImuStreamPeriod_list(struct capn_segment*, int len);
GetImuMeasurement_list new_GetImuMeasurement_list(struct capn_segment*, int len);
ImuMeasurement_list new_ImuMeasurement_list(struct capn_segment*, int len);
SetSystemStatusStreamPeriod_list new_SetSystemStatusStreamPeriod_list(struct capn_segment*, int len);
GetSystemStatus_list new_GetSystemStatus_list(struct capn_segment*, int len);
SystemStatus_list new_SystemStatus_list(struct capn_segment*, int len);
GetDistanceMeasurement_list new_GetDistanceMeasurement_list(struct capn_segment*, int len);
DistanceMeasurement_list new_DistanceMeasurement_list(struct capn_segment*, int len);
SetPositionStreamPeriod_list new_SetPositionStreamPeriod_list(struct capn_segment*, int len);
GetPosition_list new_GetPosition_list(struct capn_segment*, int len);
Position_list new_Position_list(struct capn_segment*, int len);
SetWifiConfig_list new_SetWifiConfig_list(struct capn_segment*, int len);
SetNodeId_list new_SetNodeId_list(struct capn_segment*, int len);
TextLogging_list new_TextLogging_list(struct capn_segment*, int len);
OtaUpdateBegin_list new_OtaUpdateBegin_list(struct capn_segment*, int len);
OtaUpdateCommit_list new_OtaUpdateCommit_list(struct capn_segment*, int len);
OtaUpdate_list new_OtaUpdate_list(struct capn_segment*, int len);
EnableLogging_list new_EnableLogging_list(struct capn_segment*, int len);

void read_DurinBase(struct DurinBase*, DurinBase_ptr);
void read_Reject(struct Reject*, Reject_ptr);
void read_Acknowledge(struct Acknowledge*, Acknowledge_ptr);
void read_PowerOff(struct PowerOff*, PowerOff_ptr);
void read_SetRobotVelocity(struct SetRobotVelocity*, SetRobotVelocity_ptr);
void read_SetWheelVelocity(struct SetWheelVelocity*, SetWheelVelocity_ptr);
void read_SetBuzzer(struct SetBuzzer*, SetBuzzer_ptr);
void read_SetLed(struct SetLed*, SetLed_ptr);
void read_EnableStreaming(struct EnableStreaming*, EnableStreaming_ptr);
void read_DisableStreaming(struct DisableStreaming*, DisableStreaming_ptr);
void read_SetTofStreamPeriod(struct SetTofStreamPeriod*, SetTofStreamPeriod_ptr);
void read_GetTofObservations(struct GetTofObservations*, GetTofObservations_ptr);
void read_SetTofResolution(struct SetTofResolution*, SetTofResolution_ptr);
void read_TofObservations(struct TofObservations*, TofObservations_ptr);
void read_TofObservations_TofObservation(struct TofObservations_TofObservation*, TofObservations_TofObservation_ptr);
void read_SetImuStreamPeriod(struct SetImuStreamPeriod*, SetImuStreamPeriod_ptr);
void read_GetImuMeasurement(struct GetImuMeasurement*, GetImuMeasurement_ptr);
void read_ImuMeasurement(struct ImuMeasurement*, ImuMeasurement_ptr);
void read_SetSystemStatusStreamPeriod(struct SetSystemStatusStreamPeriod*, SetSystemStatusStreamPeriod_ptr);
void read_GetSystemStatus(struct GetSystemStatus*, GetSystemStatus_ptr);
void read_SystemStatus(struct SystemStatus*, SystemStatus_ptr);
void read_GetDistanceMeasurement(struct GetDistanceMeasurement*, GetDistanceMeasurement_ptr);
void read_DistanceMeasurement(struct DistanceMeasurement*, DistanceMeasurement_ptr);
void read_SetPositionStreamPeriod(struct SetPositionStreamPeriod*, SetPositionStreamPeriod_ptr);
void read_GetPosition(struct GetPosition*, GetPosition_ptr);
void read_Position(struct Position*, Position_ptr);
void read_SetWifiConfig(struct SetWifiConfig*, SetWifiConfig_ptr);
void read_SetNodeId(struct SetNodeId*, SetNodeId_ptr);
void read_TextLogging(struct TextLogging*, TextLogging_ptr);
void read_OtaUpdateBegin(struct OtaUpdateBegin*, OtaUpdateBegin_ptr);
void read_OtaUpdateCommit(struct OtaUpdateCommit*, OtaUpdateCommit_ptr);
void read_OtaUpdate(struct OtaUpdate*, OtaUpdate_ptr);
void read_EnableLogging(struct EnableLogging*, EnableLogging_ptr);

void write_DurinBase(const struct DurinBase*, DurinBase_ptr);
void write_Reject(const struct Reject*, Reject_ptr);
void write_Acknowledge(const struct Acknowledge*, Acknowledge_ptr);
void write_PowerOff(const struct PowerOff*, PowerOff_ptr);
void write_SetRobotVelocity(const struct SetRobotVelocity*, SetRobotVelocity_ptr);
void write_SetWheelVelocity(const struct SetWheelVelocity*, SetWheelVelocity_ptr);
void write_SetBuzzer(const struct SetBuzzer*, SetBuzzer_ptr);
void write_SetLed(const struct SetLed*, SetLed_ptr);
void write_EnableStreaming(const struct EnableStreaming*, EnableStreaming_ptr);
void write_DisableStreaming(const struct DisableStreaming*, DisableStreaming_ptr);
void write_SetTofStreamPeriod(const struct SetTofStreamPeriod*, SetTofStreamPeriod_ptr);
void write_GetTofObservations(const struct GetTofObservations*, GetTofObservations_ptr);
void write_SetTofResolution(const struct SetTofResolution*, SetTofResolution_ptr);
void write_TofObservations(const struct TofObservations*, TofObservations_ptr);
void write_TofObservations_TofObservation(const struct TofObservations_TofObservation*, TofObservations_TofObservation_ptr);
void write_SetImuStreamPeriod(const struct SetImuStreamPeriod*, SetImuStreamPeriod_ptr);
void write_GetImuMeasurement(const struct GetImuMeasurement*, GetImuMeasurement_ptr);
void write_ImuMeasurement(const struct ImuMeasurement*, ImuMeasurement_ptr);
void write_SetSystemStatusStreamPeriod(const struct SetSystemStatusStreamPeriod*, SetSystemStatusStreamPeriod_ptr);
void write_GetSystemStatus(const struct GetSystemStatus*, GetSystemStatus_ptr);
void write_SystemStatus(const struct SystemStatus*, SystemStatus_ptr);
void write_GetDistanceMeasurement(const struct GetDistanceMeasurement*, GetDistanceMeasurement_ptr);
void write_DistanceMeasurement(const struct DistanceMeasurement*, DistanceMeasurement_ptr);
void write_SetPositionStreamPeriod(const struct SetPositionStreamPeriod*, SetPositionStreamPeriod_ptr);
void write_GetPosition(const struct GetPosition*, GetPosition_ptr);
void write_Position(const struct Position*, Position_ptr);
void write_SetWifiConfig(const struct SetWifiConfig*, SetWifiConfig_ptr);
void write_SetNodeId(const struct SetNodeId*, SetNodeId_ptr);
void write_TextLogging(const struct TextLogging*, TextLogging_ptr);
void write_OtaUpdateBegin(const struct OtaUpdateBegin*, OtaUpdateBegin_ptr);
void write_OtaUpdateCommit(const struct OtaUpdateCommit*, OtaUpdateCommit_ptr);
void write_OtaUpdate(const struct OtaUpdate*, OtaUpdate_ptr);
void write_EnableLogging(const struct EnableLogging*, EnableLogging_ptr);

void get_DurinBase(struct DurinBase*, DurinBase_list, int i);
void get_Reject(struct Reject*, Reject_list, int i);
void get_Acknowledge(struct Acknowledge*, Acknowledge_list, int i);
void get_PowerOff(struct PowerOff*, PowerOff_list, int i);
void get_SetRobotVelocity(struct SetRobotVelocity*, SetRobotVelocity_list, int i);
void get_SetWheelVelocity(struct SetWheelVelocity*, SetWheelVelocity_list, int i);
void get_SetBuzzer(struct SetBuzzer*, SetBuzzer_list, int i);
void get_SetLed(struct SetLed*, SetLed_list, int i);
void get_EnableStreaming(struct EnableStreaming*, EnableStreaming_list, int i);
void get_DisableStreaming(struct DisableStreaming*, DisableStreaming_list, int i);
void get_SetTofStreamPeriod(struct SetTofStreamPeriod*, SetTofStreamPeriod_list, int i);
void get_GetTofObservations(struct GetTofObservations*, GetTofObservations_list, int i);
void get_SetTofResolution(struct SetTofResolution*, SetTofResolution_list, int i);
void get_TofObservations(struct TofObservations*, TofObservations_list, int i);
void get_TofObservations_TofObservation(struct TofObservations_TofObservation*, TofObservations_TofObservation_list, int i);
void get_SetImuStreamPeriod(struct SetImuStreamPeriod*, SetImuStreamPeriod_list, int i);
void get_GetImuMeasurement(struct GetImuMeasurement*, GetImuMeasurement_list, int i);
void get_ImuMeasurement(struct ImuMeasurement*, ImuMeasurement_list, int i);
void get_SetSystemStatusStreamPeriod(struct SetSystemStatusStreamPeriod*, SetSystemStatusStreamPeriod_list, int i);
void get_GetSystemStatus(struct GetSystemStatus*, GetSystemStatus_list, int i);
void get_SystemStatus(struct SystemStatus*, SystemStatus_list, int i);
void get_GetDistanceMeasurement(struct GetDistanceMeasurement*, GetDistanceMeasurement_list, int i);
void get_DistanceMeasurement(struct DistanceMeasurement*, DistanceMeasurement_list, int i);
void get_SetPositionStreamPeriod(struct SetPositionStreamPeriod*, SetPositionStreamPeriod_list, int i);
void get_GetPosition(struct GetPosition*, GetPosition_list, int i);
void get_Position(struct Position*, Position_list, int i);
void get_SetWifiConfig(struct SetWifiConfig*, SetWifiConfig_list, int i);
void get_SetNodeId(struct SetNodeId*, SetNodeId_list, int i);
void get_TextLogging(struct TextLogging*, TextLogging_list, int i);
void get_OtaUpdateBegin(struct OtaUpdateBegin*, OtaUpdateBegin_list, int i);
void get_OtaUpdateCommit(struct OtaUpdateCommit*, OtaUpdateCommit_list, int i);
void get_OtaUpdate(struct OtaUpdate*, OtaUpdate_list, int i);
void get_EnableLogging(struct EnableLogging*, EnableLogging_list, int i);

void set_DurinBase(const struct DurinBase*, DurinBase_list, int i);
void set_Reject(const struct Reject*, Reject_list, int i);
void set_Acknowledge(const struct Acknowledge*, Acknowledge_list, int i);
void set_PowerOff(const struct PowerOff*, PowerOff_list, int i);
void set_SetRobotVelocity(const struct SetRobotVelocity*, SetRobotVelocity_list, int i);
void set_SetWheelVelocity(const struct SetWheelVelocity*, SetWheelVelocity_list, int i);
void set_SetBuzzer(const struct SetBuzzer*, SetBuzzer_list, int i);
void set_SetLed(const struct SetLed*, SetLed_list, int i);
void set_EnableStreaming(const struct EnableStreaming*, EnableStreaming_list, int i);
void set_DisableStreaming(const struct DisableStreaming*, DisableStreaming_list, int i);
void set_SetTofStreamPeriod(const struct SetTofStreamPeriod*, SetTofStreamPeriod_list, int i);
void set_GetTofObservations(const struct GetTofObservations*, GetTofObservations_list, int i);
void set_SetTofResolution(const struct SetTofResolution*, SetTofResolution_list, int i);
void set_TofObservations(const struct TofObservations*, TofObservations_list, int i);
void set_TofObservations_TofObservation(const struct TofObservations_TofObservation*, TofObservations_TofObservation_list, int i);
void set_SetImuStreamPeriod(const struct SetImuStreamPeriod*, SetImuStreamPeriod_list, int i);
void set_GetImuMeasurement(const struct GetImuMeasurement*, GetImuMeasurement_list, int i);
void set_ImuMeasurement(const struct ImuMeasurement*, ImuMeasurement_list, int i);
void set_SetSystemStatusStreamPeriod(const struct SetSystemStatusStreamPeriod*, SetSystemStatusStreamPeriod_list, int i);
void set_GetSystemStatus(const struct GetSystemStatus*, GetSystemStatus_list, int i);
void set_SystemStatus(const struct SystemStatus*, SystemStatus_list, int i);
void set_GetDistanceMeasurement(const struct GetDistanceMeasurement*, GetDistanceMeasurement_list, int i);
void set_DistanceMeasurement(const struct DistanceMeasurement*, DistanceMeasurement_list, int i);
void set_SetPositionStreamPeriod(const struct SetPositionStreamPeriod*, SetPositionStreamPeriod_list, int i);
void set_GetPosition(const struct GetPosition*, GetPosition_list, int i);
void set_Position(const struct Position*, Position_list, int i);
void set_SetWifiConfig(const struct SetWifiConfig*, SetWifiConfig_list, int i);
void set_SetNodeId(const struct SetNodeId*, SetNodeId_list, int i);
void set_TextLogging(const struct TextLogging*, TextLogging_list, int i);
void set_OtaUpdateBegin(const struct OtaUpdateBegin*, OtaUpdateBegin_list, int i);
void set_OtaUpdateCommit(const struct OtaUpdateCommit*, OtaUpdateCommit_list, int i);
void set_OtaUpdate(const struct OtaUpdate*, OtaUpdate_list, int i);
void set_EnableLogging(const struct EnableLogging*, EnableLogging_list, int i);

#ifdef __cplusplus
}
#endif
#endif
