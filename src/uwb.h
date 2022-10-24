#include "schema.capnp.h"
#include "capnp_c.h"

void init_uwb();

void build_uwb_nodes_message(UwbNodes_ptr *ptr, struct capn_segment *cs);