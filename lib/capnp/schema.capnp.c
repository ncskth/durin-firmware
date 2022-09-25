#include "schema.capnp.h"
/* AUTO GENERATED - DO NOT EDIT */
#ifdef __GNUC__
# define capnp_unused __attribute__((unused))
# define capnp_use(x) (void) x;
#else
# define capnp_unused
# define capnp_use(x)
#endif

static const capn_text capn_val0 = {0,"",0};
uint16_t streamPeriodMax = 65535;
uint16_t streamPeriodMin = 0;
uint16_t durinTcpPort = 1337;

DurinBase_ptr new_DurinBase(struct capn_segment *s) {
	DurinBase_ptr p;
	p.p = capn_new_struct(s, 8, 1);
	return p;
}
DurinBase_list new_DurinBase_list(struct capn_segment *s, int len) {
	DurinBase_list p;
	p.p = capn_new_list(s, len, 8, 1);
	return p;
}
void read_DurinBase(struct DurinBase *s capnp_unused, DurinBase_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->which = (enum DurinBase_which)(int) capn_read16(p.p, 0);
	switch (s->which) {
	case DurinBase_reject:
	case DurinBase_acknowledge:
	case DurinBase_powerOff:
	case DurinBase_setRobotVelocity:
	case DurinBase_setWheelVelocity:
	case DurinBase_setBuzzer:
	case DurinBase_setLed:
	case DurinBase_enableStreaming:
	case DurinBase_disableStreaming:
	case DurinBase_setTofStreamPeriod:
	case DurinBase_getTofObservations:
	case DurinBase_setTofResolution:
	case DurinBase_tofObservations:
	case DurinBase_setImuStreamPeriod:
	case DurinBase_getImuMeasurement:
	case DurinBase_imuMeasurement:
	case DurinBase_setSystemStatusStreamPeriod:
	case DurinBase_getSystemStatus:
	case DurinBase_systemStatus:
	case DurinBase_getDistanceMeasurement:
	case DurinBase_distanceMeasurement:
	case DurinBase_setPositionStreamPeriod:
	case DurinBase_getPosition:
	case DurinBase_position:
	case DurinBase_setWifiConfig:
	case DurinBase_setNodeId:
	case DurinBase_textLogging:
	case DurinBase_otaUpdateCommit:
	case DurinBase_otaUpdate:
	case DurinBase_enableLogging:
	case DurinBase_otaUpdateBegin:
		s->otaUpdateBegin.p = capn_getp(p.p, 0, 0);
		break;
	default:
		break;
	}
}
void write_DurinBase(const struct DurinBase *s capnp_unused, DurinBase_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->which);
	switch (s->which) {
	case DurinBase_reject:
	case DurinBase_acknowledge:
	case DurinBase_powerOff:
	case DurinBase_setRobotVelocity:
	case DurinBase_setWheelVelocity:
	case DurinBase_setBuzzer:
	case DurinBase_setLed:
	case DurinBase_enableStreaming:
	case DurinBase_disableStreaming:
	case DurinBase_setTofStreamPeriod:
	case DurinBase_getTofObservations:
	case DurinBase_setTofResolution:
	case DurinBase_tofObservations:
	case DurinBase_setImuStreamPeriod:
	case DurinBase_getImuMeasurement:
	case DurinBase_imuMeasurement:
	case DurinBase_setSystemStatusStreamPeriod:
	case DurinBase_getSystemStatus:
	case DurinBase_systemStatus:
	case DurinBase_getDistanceMeasurement:
	case DurinBase_distanceMeasurement:
	case DurinBase_setPositionStreamPeriod:
	case DurinBase_getPosition:
	case DurinBase_position:
	case DurinBase_setWifiConfig:
	case DurinBase_setNodeId:
	case DurinBase_textLogging:
	case DurinBase_otaUpdateCommit:
	case DurinBase_otaUpdate:
	case DurinBase_enableLogging:
	case DurinBase_otaUpdateBegin:
		capn_setp(p.p, 0, s->otaUpdateBegin.p);
		break;
	default:
		break;
	}
}
void get_DurinBase(struct DurinBase *s, DurinBase_list l, int i) {
	DurinBase_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_DurinBase(s, p);
}
void set_DurinBase(const struct DurinBase *s, DurinBase_list l, int i) {
	DurinBase_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_DurinBase(s, p);
}

Reject_ptr new_Reject(struct capn_segment *s) {
	Reject_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
Reject_list new_Reject_list(struct capn_segment *s, int len) {
	Reject_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_Reject(struct Reject *s capnp_unused, Reject_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_Reject(const struct Reject *s capnp_unused, Reject_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_Reject(struct Reject *s, Reject_list l, int i) {
	Reject_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_Reject(s, p);
}
void set_Reject(const struct Reject *s, Reject_list l, int i) {
	Reject_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_Reject(s, p);
}

Acknowledge_ptr new_Acknowledge(struct capn_segment *s) {
	Acknowledge_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
Acknowledge_list new_Acknowledge_list(struct capn_segment *s, int len) {
	Acknowledge_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_Acknowledge(struct Acknowledge *s capnp_unused, Acknowledge_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_Acknowledge(const struct Acknowledge *s capnp_unused, Acknowledge_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_Acknowledge(struct Acknowledge *s, Acknowledge_list l, int i) {
	Acknowledge_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_Acknowledge(s, p);
}
void set_Acknowledge(const struct Acknowledge *s, Acknowledge_list l, int i) {
	Acknowledge_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_Acknowledge(s, p);
}

PowerOff_ptr new_PowerOff(struct capn_segment *s) {
	PowerOff_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
PowerOff_list new_PowerOff_list(struct capn_segment *s, int len) {
	PowerOff_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_PowerOff(struct PowerOff *s capnp_unused, PowerOff_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_PowerOff(const struct PowerOff *s capnp_unused, PowerOff_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_PowerOff(struct PowerOff *s, PowerOff_list l, int i) {
	PowerOff_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_PowerOff(s, p);
}
void set_PowerOff(const struct PowerOff *s, PowerOff_list l, int i) {
	PowerOff_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_PowerOff(s, p);
}

SetRobotVelocity_ptr new_SetRobotVelocity(struct capn_segment *s) {
	SetRobotVelocity_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetRobotVelocity_list new_SetRobotVelocity_list(struct capn_segment *s, int len) {
	SetRobotVelocity_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetRobotVelocity(struct SetRobotVelocity *s capnp_unused, SetRobotVelocity_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->velocityXMms = (int16_t) ((int16_t)capn_read16(p.p, 0));
	s->velocityYMms = (int16_t) ((int16_t)capn_read16(p.p, 2));
	s->rotationDegs = (int16_t) ((int16_t)capn_read16(p.p, 4));
}
void write_SetRobotVelocity(const struct SetRobotVelocity *s capnp_unused, SetRobotVelocity_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, (uint16_t) (s->velocityXMms));
	capn_write16(p.p, 2, (uint16_t) (s->velocityYMms));
	capn_write16(p.p, 4, (uint16_t) (s->rotationDegs));
}
void get_SetRobotVelocity(struct SetRobotVelocity *s, SetRobotVelocity_list l, int i) {
	SetRobotVelocity_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetRobotVelocity(s, p);
}
void set_SetRobotVelocity(const struct SetRobotVelocity *s, SetRobotVelocity_list l, int i) {
	SetRobotVelocity_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetRobotVelocity(s, p);
}

SetWheelVelocity_ptr new_SetWheelVelocity(struct capn_segment *s) {
	SetWheelVelocity_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetWheelVelocity_list new_SetWheelVelocity_list(struct capn_segment *s, int len) {
	SetWheelVelocity_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetWheelVelocity(struct SetWheelVelocity *s capnp_unused, SetWheelVelocity_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->wheelFrontLeftMms = (int16_t) ((int16_t)capn_read16(p.p, 0));
	s->wheelFrontRightMms = (int16_t) ((int16_t)capn_read16(p.p, 2));
	s->wheelBackLeftMms = (int16_t) ((int16_t)capn_read16(p.p, 4));
	s->wheelBackRightMms = (int16_t) ((int16_t)capn_read16(p.p, 6));
}
void write_SetWheelVelocity(const struct SetWheelVelocity *s capnp_unused, SetWheelVelocity_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, (uint16_t) (s->wheelFrontLeftMms));
	capn_write16(p.p, 2, (uint16_t) (s->wheelFrontRightMms));
	capn_write16(p.p, 4, (uint16_t) (s->wheelBackLeftMms));
	capn_write16(p.p, 6, (uint16_t) (s->wheelBackRightMms));
}
void get_SetWheelVelocity(struct SetWheelVelocity *s, SetWheelVelocity_list l, int i) {
	SetWheelVelocity_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetWheelVelocity(s, p);
}
void set_SetWheelVelocity(const struct SetWheelVelocity *s, SetWheelVelocity_list l, int i) {
	SetWheelVelocity_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetWheelVelocity(s, p);
}

SetBuzzer_ptr new_SetBuzzer(struct capn_segment *s) {
	SetBuzzer_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetBuzzer_list new_SetBuzzer_list(struct capn_segment *s, int len) {
	SetBuzzer_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetBuzzer(struct SetBuzzer *s capnp_unused, SetBuzzer_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->enabled = (capn_read8(p.p, 0) & 1) != 0;
}
void write_SetBuzzer(const struct SetBuzzer *s capnp_unused, SetBuzzer_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write1(p.p, 0, s->enabled != 0);
}
void get_SetBuzzer(struct SetBuzzer *s, SetBuzzer_list l, int i) {
	SetBuzzer_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetBuzzer(s, p);
}
void set_SetBuzzer(const struct SetBuzzer *s, SetBuzzer_list l, int i) {
	SetBuzzer_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetBuzzer(s, p);
}

SetLed_ptr new_SetLed(struct capn_segment *s) {
	SetLed_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetLed_list new_SetLed_list(struct capn_segment *s, int len) {
	SetLed_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetLed(struct SetLed *s capnp_unused, SetLed_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->ledR = capn_read8(p.p, 0);
	s->ledG = capn_read8(p.p, 1);
	s->ledB = capn_read8(p.p, 2);
}
void write_SetLed(const struct SetLed *s capnp_unused, SetLed_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write8(p.p, 0, s->ledR);
	capn_write8(p.p, 1, s->ledG);
	capn_write8(p.p, 2, s->ledB);
}
void get_SetLed(struct SetLed *s, SetLed_list l, int i) {
	SetLed_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetLed(s, p);
}
void set_SetLed(const struct SetLed *s, SetLed_list l, int i) {
	SetLed_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetLed(s, p);
}

EnableStreaming_ptr new_EnableStreaming(struct capn_segment *s) {
	EnableStreaming_ptr p;
	p.p = capn_new_struct(s, 8, 1);
	return p;
}
EnableStreaming_list new_EnableStreaming_list(struct capn_segment *s, int len) {
	EnableStreaming_list p;
	p.p = capn_new_list(s, len, 8, 1);
	return p;
}
void read_EnableStreaming(struct EnableStreaming *s capnp_unused, EnableStreaming_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->destination_which = (enum EnableStreaming_destination_which)(int) capn_read16(p.p, 0);
	switch (s->destination_which) {
	case EnableStreaming_destination_udpOnly:
		s->destination.udpOnly.ip.p = capn_getp(p.p, 0, 0);
		s->destination.udpOnly.port = capn_read16(p.p, 2);
		break;
	case EnableStreaming_destination_uartAndUdp:
		s->destination.uartAndUdp.ip.p = capn_getp(p.p, 0, 0);
		s->destination.uartAndUdp.port = capn_read16(p.p, 2);
		break;
	default:
		break;
	}
}
void write_EnableStreaming(const struct EnableStreaming *s capnp_unused, EnableStreaming_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->destination_which);
	switch (s->destination_which) {
	case EnableStreaming_destination_udpOnly:
		capn_setp(p.p, 0, s->destination.udpOnly.ip.p);
		capn_write16(p.p, 2, s->destination.udpOnly.port);
		break;
	case EnableStreaming_destination_uartAndUdp:
		capn_setp(p.p, 0, s->destination.uartAndUdp.ip.p);
		capn_write16(p.p, 2, s->destination.uartAndUdp.port);
		break;
	default:
		break;
	}
}
void get_EnableStreaming(struct EnableStreaming *s, EnableStreaming_list l, int i) {
	EnableStreaming_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_EnableStreaming(s, p);
}
void set_EnableStreaming(const struct EnableStreaming *s, EnableStreaming_list l, int i) {
	EnableStreaming_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_EnableStreaming(s, p);
}

DisableStreaming_ptr new_DisableStreaming(struct capn_segment *s) {
	DisableStreaming_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
DisableStreaming_list new_DisableStreaming_list(struct capn_segment *s, int len) {
	DisableStreaming_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_DisableStreaming(struct DisableStreaming *s capnp_unused, DisableStreaming_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_DisableStreaming(const struct DisableStreaming *s capnp_unused, DisableStreaming_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_DisableStreaming(struct DisableStreaming *s, DisableStreaming_list l, int i) {
	DisableStreaming_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_DisableStreaming(s, p);
}
void set_DisableStreaming(const struct DisableStreaming *s, DisableStreaming_list l, int i) {
	DisableStreaming_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_DisableStreaming(s, p);
}

SetTofStreamPeriod_ptr new_SetTofStreamPeriod(struct capn_segment *s) {
	SetTofStreamPeriod_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetTofStreamPeriod_list new_SetTofStreamPeriod_list(struct capn_segment *s, int len) {
	SetTofStreamPeriod_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetTofStreamPeriod(struct SetTofStreamPeriod *s capnp_unused, SetTofStreamPeriod_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->periodMs = capn_read16(p.p, 0);
}
void write_SetTofStreamPeriod(const struct SetTofStreamPeriod *s capnp_unused, SetTofStreamPeriod_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->periodMs);
}
void get_SetTofStreamPeriod(struct SetTofStreamPeriod *s, SetTofStreamPeriod_list l, int i) {
	SetTofStreamPeriod_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetTofStreamPeriod(s, p);
}
void set_SetTofStreamPeriod(const struct SetTofStreamPeriod *s, SetTofStreamPeriod_list l, int i) {
	SetTofStreamPeriod_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetTofStreamPeriod(s, p);
}

GetTofObservations_ptr new_GetTofObservations(struct capn_segment *s) {
	GetTofObservations_ptr p;
	p.p = capn_new_struct(s, 0, 1);
	return p;
}
GetTofObservations_list new_GetTofObservations_list(struct capn_segment *s, int len) {
	GetTofObservations_list p;
	p.p = capn_new_list(s, len, 0, 1);
	return p;
}
void read_GetTofObservations(struct GetTofObservations *s capnp_unused, GetTofObservations_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->ids.p = capn_getp(p.p, 0, 0);
}
void write_GetTofObservations(const struct GetTofObservations *s capnp_unused, GetTofObservations_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_setp(p.p, 0, s->ids.p);
}
void get_GetTofObservations(struct GetTofObservations *s, GetTofObservations_list l, int i) {
	GetTofObservations_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_GetTofObservations(s, p);
}
void set_GetTofObservations(const struct GetTofObservations *s, GetTofObservations_list l, int i) {
	GetTofObservations_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_GetTofObservations(s, p);
}

SetTofResolution_ptr new_SetTofResolution(struct capn_segment *s) {
	SetTofResolution_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetTofResolution_list new_SetTofResolution_list(struct capn_segment *s, int len) {
	SetTofResolution_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetTofResolution(struct SetTofResolution *s capnp_unused, SetTofResolution_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->resolution = (enum SetTofResolution_TofResolutions)(int) capn_read16(p.p, 0);
}
void write_SetTofResolution(const struct SetTofResolution *s capnp_unused, SetTofResolution_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, (uint16_t) (s->resolution));
}
void get_SetTofResolution(struct SetTofResolution *s, SetTofResolution_list l, int i) {
	SetTofResolution_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetTofResolution(s, p);
}
void set_SetTofResolution(const struct SetTofResolution *s, SetTofResolution_list l, int i) {
	SetTofResolution_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetTofResolution(s, p);
}

TofObservations_ptr new_TofObservations(struct capn_segment *s) {
	TofObservations_ptr p;
	p.p = capn_new_struct(s, 0, 1);
	return p;
}
TofObservations_list new_TofObservations_list(struct capn_segment *s, int len) {
	TofObservations_list p;
	p.p = capn_new_list(s, len, 0, 1);
	return p;
}
void read_TofObservations(struct TofObservations *s capnp_unused, TofObservations_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->observations.p = capn_getp(p.p, 0, 0);
}
void write_TofObservations(const struct TofObservations *s capnp_unused, TofObservations_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_setp(p.p, 0, s->observations.p);
}
void get_TofObservations(struct TofObservations *s, TofObservations_list l, int i) {
	TofObservations_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_TofObservations(s, p);
}
void set_TofObservations(const struct TofObservations *s, TofObservations_list l, int i) {
	TofObservations_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_TofObservations(s, p);
}

TofObservations_TofObservation_ptr new_TofObservations_TofObservation(struct capn_segment *s) {
	TofObservations_TofObservation_ptr p;
	p.p = capn_new_struct(s, 8, 1);
	return p;
}
TofObservations_TofObservation_list new_TofObservations_TofObservation_list(struct capn_segment *s, int len) {
	TofObservations_TofObservation_list p;
	p.p = capn_new_list(s, len, 8, 1);
	return p;
}
void read_TofObservations_TofObservation(struct TofObservations_TofObservation *s capnp_unused, TofObservations_TofObservation_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->id = capn_read8(p.p, 0);
	s->ranges.p = capn_getp(p.p, 0, 0);
}
void write_TofObservations_TofObservation(const struct TofObservations_TofObservation *s capnp_unused, TofObservations_TofObservation_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write8(p.p, 0, s->id);
	capn_setp(p.p, 0, s->ranges.p);
}
void get_TofObservations_TofObservation(struct TofObservations_TofObservation *s, TofObservations_TofObservation_list l, int i) {
	TofObservations_TofObservation_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_TofObservations_TofObservation(s, p);
}
void set_TofObservations_TofObservation(const struct TofObservations_TofObservation *s, TofObservations_TofObservation_list l, int i) {
	TofObservations_TofObservation_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_TofObservations_TofObservation(s, p);
}

SetImuStreamPeriod_ptr new_SetImuStreamPeriod(struct capn_segment *s) {
	SetImuStreamPeriod_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetImuStreamPeriod_list new_SetImuStreamPeriod_list(struct capn_segment *s, int len) {
	SetImuStreamPeriod_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetImuStreamPeriod(struct SetImuStreamPeriod *s capnp_unused, SetImuStreamPeriod_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->periodMs = capn_read16(p.p, 0);
}
void write_SetImuStreamPeriod(const struct SetImuStreamPeriod *s capnp_unused, SetImuStreamPeriod_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->periodMs);
}
void get_SetImuStreamPeriod(struct SetImuStreamPeriod *s, SetImuStreamPeriod_list l, int i) {
	SetImuStreamPeriod_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetImuStreamPeriod(s, p);
}
void set_SetImuStreamPeriod(const struct SetImuStreamPeriod *s, SetImuStreamPeriod_list l, int i) {
	SetImuStreamPeriod_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetImuStreamPeriod(s, p);
}

GetImuMeasurement_ptr new_GetImuMeasurement(struct capn_segment *s) {
	GetImuMeasurement_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
GetImuMeasurement_list new_GetImuMeasurement_list(struct capn_segment *s, int len) {
	GetImuMeasurement_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_GetImuMeasurement(struct GetImuMeasurement *s capnp_unused, GetImuMeasurement_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_GetImuMeasurement(const struct GetImuMeasurement *s capnp_unused, GetImuMeasurement_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_GetImuMeasurement(struct GetImuMeasurement *s, GetImuMeasurement_list l, int i) {
	GetImuMeasurement_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_GetImuMeasurement(s, p);
}
void set_GetImuMeasurement(const struct GetImuMeasurement *s, GetImuMeasurement_list l, int i) {
	GetImuMeasurement_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_GetImuMeasurement(s, p);
}

ImuMeasurement_ptr new_ImuMeasurement(struct capn_segment *s) {
	ImuMeasurement_ptr p;
	p.p = capn_new_struct(s, 40, 0);
	return p;
}
ImuMeasurement_list new_ImuMeasurement_list(struct capn_segment *s, int len) {
	ImuMeasurement_list p;
	p.p = capn_new_list(s, len, 40, 0);
	return p;
}
void read_ImuMeasurement(struct ImuMeasurement *s capnp_unused, ImuMeasurement_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->accelerometerXG = capn_to_f32(capn_read32(p.p, 0));
	s->accelerometerYG = capn_to_f32(capn_read32(p.p, 4));
	s->accelerometerZG = capn_to_f32(capn_read32(p.p, 8));
	s->gyroscopeXRads = capn_to_f32(capn_read32(p.p, 12));
	s->gyroscopeYRads = capn_to_f32(capn_read32(p.p, 16));
	s->gyroscopeZRads = capn_to_f32(capn_read32(p.p, 20));
	s->magnetometerXUt = capn_to_f32(capn_read32(p.p, 24));
	s->magnetometerYUt = capn_to_f32(capn_read32(p.p, 28));
	s->magnetometerZUt = capn_to_f32(capn_read32(p.p, 32));
}
void write_ImuMeasurement(const struct ImuMeasurement *s capnp_unused, ImuMeasurement_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write32(p.p, 0, capn_from_f32(s->accelerometerXG));
	capn_write32(p.p, 4, capn_from_f32(s->accelerometerYG));
	capn_write32(p.p, 8, capn_from_f32(s->accelerometerZG));
	capn_write32(p.p, 12, capn_from_f32(s->gyroscopeXRads));
	capn_write32(p.p, 16, capn_from_f32(s->gyroscopeYRads));
	capn_write32(p.p, 20, capn_from_f32(s->gyroscopeZRads));
	capn_write32(p.p, 24, capn_from_f32(s->magnetometerXUt));
	capn_write32(p.p, 28, capn_from_f32(s->magnetometerYUt));
	capn_write32(p.p, 32, capn_from_f32(s->magnetometerZUt));
}
void get_ImuMeasurement(struct ImuMeasurement *s, ImuMeasurement_list l, int i) {
	ImuMeasurement_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_ImuMeasurement(s, p);
}
void set_ImuMeasurement(const struct ImuMeasurement *s, ImuMeasurement_list l, int i) {
	ImuMeasurement_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_ImuMeasurement(s, p);
}

SetSystemStatusStreamPeriod_ptr new_SetSystemStatusStreamPeriod(struct capn_segment *s) {
	SetSystemStatusStreamPeriod_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetSystemStatusStreamPeriod_list new_SetSystemStatusStreamPeriod_list(struct capn_segment *s, int len) {
	SetSystemStatusStreamPeriod_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetSystemStatusStreamPeriod(struct SetSystemStatusStreamPeriod *s capnp_unused, SetSystemStatusStreamPeriod_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->periodMs = capn_read16(p.p, 0);
}
void write_SetSystemStatusStreamPeriod(const struct SetSystemStatusStreamPeriod *s capnp_unused, SetSystemStatusStreamPeriod_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->periodMs);
}
void get_SetSystemStatusStreamPeriod(struct SetSystemStatusStreamPeriod *s, SetSystemStatusStreamPeriod_list l, int i) {
	SetSystemStatusStreamPeriod_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetSystemStatusStreamPeriod(s, p);
}
void set_SetSystemStatusStreamPeriod(const struct SetSystemStatusStreamPeriod *s, SetSystemStatusStreamPeriod_list l, int i) {
	SetSystemStatusStreamPeriod_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetSystemStatusStreamPeriod(s, p);
}

GetSystemStatus_ptr new_GetSystemStatus(struct capn_segment *s) {
	GetSystemStatus_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
GetSystemStatus_list new_GetSystemStatus_list(struct capn_segment *s, int len) {
	GetSystemStatus_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_GetSystemStatus(struct GetSystemStatus *s capnp_unused, GetSystemStatus_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_GetSystemStatus(const struct GetSystemStatus *s capnp_unused, GetSystemStatus_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_GetSystemStatus(struct GetSystemStatus *s, GetSystemStatus_list l, int i) {
	GetSystemStatus_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_GetSystemStatus(s, p);
}
void set_GetSystemStatus(const struct GetSystemStatus *s, GetSystemStatus_list l, int i) {
	GetSystemStatus_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_GetSystemStatus(s, p);
}

SystemStatus_ptr new_SystemStatus(struct capn_segment *s) {
	SystemStatus_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SystemStatus_list new_SystemStatus_list(struct capn_segment *s, int len) {
	SystemStatus_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SystemStatus(struct SystemStatus *s capnp_unused, SystemStatus_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->batteryMv = capn_read16(p.p, 0);
	s->batteryPercent = capn_read8(p.p, 2);
}
void write_SystemStatus(const struct SystemStatus *s capnp_unused, SystemStatus_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->batteryMv);
	capn_write8(p.p, 2, s->batteryPercent);
}
void get_SystemStatus(struct SystemStatus *s, SystemStatus_list l, int i) {
	SystemStatus_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SystemStatus(s, p);
}
void set_SystemStatus(const struct SystemStatus *s, SystemStatus_list l, int i) {
	SystemStatus_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SystemStatus(s, p);
}

GetDistanceMeasurement_ptr new_GetDistanceMeasurement(struct capn_segment *s) {
	GetDistanceMeasurement_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
GetDistanceMeasurement_list new_GetDistanceMeasurement_list(struct capn_segment *s, int len) {
	GetDistanceMeasurement_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_GetDistanceMeasurement(struct GetDistanceMeasurement *s capnp_unused, GetDistanceMeasurement_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->nodeId = capn_read8(p.p, 0);
}
void write_GetDistanceMeasurement(const struct GetDistanceMeasurement *s capnp_unused, GetDistanceMeasurement_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write8(p.p, 0, s->nodeId);
}
void get_GetDistanceMeasurement(struct GetDistanceMeasurement *s, GetDistanceMeasurement_list l, int i) {
	GetDistanceMeasurement_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_GetDistanceMeasurement(s, p);
}
void set_GetDistanceMeasurement(const struct GetDistanceMeasurement *s, GetDistanceMeasurement_list l, int i) {
	GetDistanceMeasurement_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_GetDistanceMeasurement(s, p);
}

DistanceMeasurement_ptr new_DistanceMeasurement(struct capn_segment *s) {
	DistanceMeasurement_ptr p;
	p.p = capn_new_struct(s, 24, 0);
	return p;
}
DistanceMeasurement_list new_DistanceMeasurement_list(struct capn_segment *s, int len) {
	DistanceMeasurement_list p;
	p.p = capn_new_list(s, len, 24, 0);
	return p;
}
void read_DistanceMeasurement(struct DistanceMeasurement *s capnp_unused, DistanceMeasurement_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->nodeId = capn_read8(p.p, 0);
	s->distanceMm = capn_read32(p.p, 4);
	s->position_which = (enum DistanceMeasurement_position_which)(int) capn_read16(p.p, 2);
	switch (s->position_which) {
	case DistanceMeasurement_position_vector:
		s->position.vector.x = (int32_t) ((int32_t)capn_read32(p.p, 8));
		s->position.vector.y = (int32_t) ((int32_t)capn_read32(p.p, 12));
		s->position.vector.z = (int32_t) ((int32_t)capn_read32(p.p, 16));
		break;
	default:
		break;
	}
}
void write_DistanceMeasurement(const struct DistanceMeasurement *s capnp_unused, DistanceMeasurement_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write8(p.p, 0, s->nodeId);
	capn_write32(p.p, 4, s->distanceMm);
	capn_write16(p.p, 2, s->position_which);
	switch (s->position_which) {
	case DistanceMeasurement_position_vector:
		capn_write32(p.p, 8, (uint32_t) (s->position.vector.x));
		capn_write32(p.p, 12, (uint32_t) (s->position.vector.y));
		capn_write32(p.p, 16, (uint32_t) (s->position.vector.z));
		break;
	default:
		break;
	}
}
void get_DistanceMeasurement(struct DistanceMeasurement *s, DistanceMeasurement_list l, int i) {
	DistanceMeasurement_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_DistanceMeasurement(s, p);
}
void set_DistanceMeasurement(const struct DistanceMeasurement *s, DistanceMeasurement_list l, int i) {
	DistanceMeasurement_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_DistanceMeasurement(s, p);
}

SetPositionStreamPeriod_ptr new_SetPositionStreamPeriod(struct capn_segment *s) {
	SetPositionStreamPeriod_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetPositionStreamPeriod_list new_SetPositionStreamPeriod_list(struct capn_segment *s, int len) {
	SetPositionStreamPeriod_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetPositionStreamPeriod(struct SetPositionStreamPeriod *s capnp_unused, SetPositionStreamPeriod_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->periodMs = capn_read16(p.p, 0);
}
void write_SetPositionStreamPeriod(const struct SetPositionStreamPeriod *s capnp_unused, SetPositionStreamPeriod_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->periodMs);
}
void get_SetPositionStreamPeriod(struct SetPositionStreamPeriod *s, SetPositionStreamPeriod_list l, int i) {
	SetPositionStreamPeriod_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetPositionStreamPeriod(s, p);
}
void set_SetPositionStreamPeriod(const struct SetPositionStreamPeriod *s, SetPositionStreamPeriod_list l, int i) {
	SetPositionStreamPeriod_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetPositionStreamPeriod(s, p);
}

GetPosition_ptr new_GetPosition(struct capn_segment *s) {
	GetPosition_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
GetPosition_list new_GetPosition_list(struct capn_segment *s, int len) {
	GetPosition_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_GetPosition(struct GetPosition *s capnp_unused, GetPosition_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_GetPosition(const struct GetPosition *s capnp_unused, GetPosition_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_GetPosition(struct GetPosition *s, GetPosition_list l, int i) {
	GetPosition_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_GetPosition(s, p);
}
void set_GetPosition(const struct GetPosition *s, GetPosition_list l, int i) {
	GetPosition_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_GetPosition(s, p);
}

Position_ptr new_Position(struct capn_segment *s) {
	Position_ptr p;
	p.p = capn_new_struct(s, 16, 0);
	return p;
}
Position_list new_Position_list(struct capn_segment *s, int len) {
	Position_list p;
	p.p = capn_new_list(s, len, 16, 0);
	return p;
}
void read_Position(struct Position *s capnp_unused, Position_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->which = (enum Position_which)(int) capn_read16(p.p, 0);
	switch (s->which) {
	case Position_vector:
		s->vector.x = (int32_t) ((int32_t)capn_read32(p.p, 4));
		s->vector.y = (int32_t) ((int32_t)capn_read32(p.p, 8));
		s->vector.z = (int32_t) ((int32_t)capn_read32(p.p, 12));
		break;
	default:
		break;
	}
}
void write_Position(const struct Position *s capnp_unused, Position_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->which);
	switch (s->which) {
	case Position_vector:
		capn_write32(p.p, 4, (uint32_t) (s->vector.x));
		capn_write32(p.p, 8, (uint32_t) (s->vector.y));
		capn_write32(p.p, 12, (uint32_t) (s->vector.z));
		break;
	default:
		break;
	}
}
void get_Position(struct Position *s, Position_list l, int i) {
	Position_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_Position(s, p);
}
void set_Position(const struct Position *s, Position_list l, int i) {
	Position_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_Position(s, p);
}

SetWifiConfig_ptr new_SetWifiConfig(struct capn_segment *s) {
	SetWifiConfig_ptr p;
	p.p = capn_new_struct(s, 0, 2);
	return p;
}
SetWifiConfig_list new_SetWifiConfig_list(struct capn_segment *s, int len) {
	SetWifiConfig_list p;
	p.p = capn_new_list(s, len, 0, 2);
	return p;
}
void read_SetWifiConfig(struct SetWifiConfig *s capnp_unused, SetWifiConfig_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->ssid = capn_get_text(p.p, 0, capn_val0);
	s->password = capn_get_text(p.p, 1, capn_val0);
}
void write_SetWifiConfig(const struct SetWifiConfig *s capnp_unused, SetWifiConfig_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_set_text(p.p, 0, s->ssid);
	capn_set_text(p.p, 1, s->password);
}
void get_SetWifiConfig(struct SetWifiConfig *s, SetWifiConfig_list l, int i) {
	SetWifiConfig_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetWifiConfig(s, p);
}
void set_SetWifiConfig(const struct SetWifiConfig *s, SetWifiConfig_list l, int i) {
	SetWifiConfig_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetWifiConfig(s, p);
}

SetNodeId_ptr new_SetNodeId(struct capn_segment *s) {
	SetNodeId_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
SetNodeId_list new_SetNodeId_list(struct capn_segment *s, int len) {
	SetNodeId_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_SetNodeId(struct SetNodeId *s capnp_unused, SetNodeId_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->nodeId = capn_read8(p.p, 0);
}
void write_SetNodeId(const struct SetNodeId *s capnp_unused, SetNodeId_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write8(p.p, 0, s->nodeId);
}
void get_SetNodeId(struct SetNodeId *s, SetNodeId_list l, int i) {
	SetNodeId_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_SetNodeId(s, p);
}
void set_SetNodeId(const struct SetNodeId *s, SetNodeId_list l, int i) {
	SetNodeId_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_SetNodeId(s, p);
}

TextLogging_ptr new_TextLogging(struct capn_segment *s) {
	TextLogging_ptr p;
	p.p = capn_new_struct(s, 0, 1);
	return p;
}
TextLogging_list new_TextLogging_list(struct capn_segment *s, int len) {
	TextLogging_list p;
	p.p = capn_new_list(s, len, 0, 1);
	return p;
}
void read_TextLogging(struct TextLogging *s capnp_unused, TextLogging_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->log = capn_get_text(p.p, 0, capn_val0);
}
void write_TextLogging(const struct TextLogging *s capnp_unused, TextLogging_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_set_text(p.p, 0, s->log);
}
void get_TextLogging(struct TextLogging *s, TextLogging_list l, int i) {
	TextLogging_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_TextLogging(s, p);
}
void set_TextLogging(const struct TextLogging *s, TextLogging_list l, int i) {
	TextLogging_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_TextLogging(s, p);
}

OtaUpdateBegin_ptr new_OtaUpdateBegin(struct capn_segment *s) {
	OtaUpdateBegin_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
OtaUpdateBegin_list new_OtaUpdateBegin_list(struct capn_segment *s, int len) {
	OtaUpdateBegin_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_OtaUpdateBegin(struct OtaUpdateBegin *s capnp_unused, OtaUpdateBegin_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_OtaUpdateBegin(const struct OtaUpdateBegin *s capnp_unused, OtaUpdateBegin_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_OtaUpdateBegin(struct OtaUpdateBegin *s, OtaUpdateBegin_list l, int i) {
	OtaUpdateBegin_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_OtaUpdateBegin(s, p);
}
void set_OtaUpdateBegin(const struct OtaUpdateBegin *s, OtaUpdateBegin_list l, int i) {
	OtaUpdateBegin_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_OtaUpdateBegin(s, p);
}

OtaUpdateCommit_ptr new_OtaUpdateCommit(struct capn_segment *s) {
	OtaUpdateCommit_ptr p;
	p.p = capn_new_struct(s, 0, 0);
	return p;
}
OtaUpdateCommit_list new_OtaUpdateCommit_list(struct capn_segment *s, int len) {
	OtaUpdateCommit_list p;
	p.p = capn_new_list(s, len, 0, 0);
	return p;
}
void read_OtaUpdateCommit(struct OtaUpdateCommit *s capnp_unused, OtaUpdateCommit_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void write_OtaUpdateCommit(const struct OtaUpdateCommit *s capnp_unused, OtaUpdateCommit_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
}
void get_OtaUpdateCommit(struct OtaUpdateCommit *s, OtaUpdateCommit_list l, int i) {
	OtaUpdateCommit_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_OtaUpdateCommit(s, p);
}
void set_OtaUpdateCommit(const struct OtaUpdateCommit *s, OtaUpdateCommit_list l, int i) {
	OtaUpdateCommit_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_OtaUpdateCommit(s, p);
}

OtaUpdate_ptr new_OtaUpdate(struct capn_segment *s) {
	OtaUpdate_ptr p;
	p.p = capn_new_struct(s, 0, 1);
	return p;
}
OtaUpdate_list new_OtaUpdate_list(struct capn_segment *s, int len) {
	OtaUpdate_list p;
	p.p = capn_new_list(s, len, 0, 1);
	return p;
}
void read_OtaUpdate(struct OtaUpdate *s capnp_unused, OtaUpdate_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->data = capn_get_data(p.p, 0);
}
void write_OtaUpdate(const struct OtaUpdate *s capnp_unused, OtaUpdate_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_setp(p.p, 0, s->data.p);
}
void get_OtaUpdate(struct OtaUpdate *s, OtaUpdate_list l, int i) {
	OtaUpdate_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_OtaUpdate(s, p);
}
void set_OtaUpdate(const struct OtaUpdate *s, OtaUpdate_list l, int i) {
	OtaUpdate_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_OtaUpdate(s, p);
}

EnableLogging_ptr new_EnableLogging(struct capn_segment *s) {
	EnableLogging_ptr p;
	p.p = capn_new_struct(s, 8, 0);
	return p;
}
EnableLogging_list new_EnableLogging_list(struct capn_segment *s, int len) {
	EnableLogging_list p;
	p.p = capn_new_list(s, len, 8, 0);
	return p;
}
void read_EnableLogging(struct EnableLogging *s capnp_unused, EnableLogging_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	s->which = (enum EnableLogging_which)(int) capn_read16(p.p, 0);
	switch (s->which) {
	default:
		break;
	}
}
void write_EnableLogging(const struct EnableLogging *s capnp_unused, EnableLogging_ptr p) {
	capn_resolve(&p.p);
	capnp_use(s);
	capn_write16(p.p, 0, s->which);
	switch (s->which) {
	default:
		break;
	}
}
void get_EnableLogging(struct EnableLogging *s, EnableLogging_list l, int i) {
	EnableLogging_ptr p;
	p.p = capn_getp(l.p, i, 0);
	read_EnableLogging(s, p);
}
void set_EnableLogging(const struct EnableLogging *s, EnableLogging_list l, int i) {
	EnableLogging_ptr p;
	p.p = capn_getp(l.p, i, 0);
	write_EnableLogging(s, p);
}