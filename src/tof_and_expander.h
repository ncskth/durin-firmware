#include "pt.h"
#include "schema.capnp.h"
#include "capnp_c.h"

void init_tof_and_expander();

void update_tof_and_expander(struct pt *pt);

void set_tof_resolution(enum TofResolutions resolution);
void build_tof_message(TofObservations_ptr *ptr, struct capn_segment *cs, uint8_t *to_send);