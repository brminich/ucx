/**
 * Copyright (C) Mellanox Technologies Ltd. 2016.  ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */

#ifndef DT_COMMON_H
#define DT_COMMON_H

#include "dt_contig.h"
#include "dt_iov.h"
#include "dt_generic.h"
#include "dt_struct.h"

#include <ucp/core/ucp_types.h>
#include <uct/api/uct.h>
#include <ucp/api/ucp.h>


/**
 * Memory registration state of a buffer/operation
 */
typedef struct ucp_dt_reg {
    ucp_md_map_t                  md_map;    /* Map of used memory domains */
    uct_mem_h                     memh[UCP_MAX_OP_MDS];
} ucp_dt_reg_t;


#endif // DT_COMMON_H
