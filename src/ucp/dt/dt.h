/**
 * Copyright (C) Mellanox Technologies Ltd. 2016.  ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */


#ifndef UCP_DT_H_
#define UCP_DT_H_

#include "dt_contig.h"
#include "dt_iov.h"
#include "dt_generic.h"
#include "dt_struct.h"
#include "dt_common.h"

#include <ucp/core/ucp_types.h>
#include <uct/api/uct.h>
#include <ucp/api/ucp.h>


/**
 * State of progressing sent/receive operation on a datatype.
 */
typedef struct ucp_dt_state {
    size_t                        offset;  /* Total offset in overall payload. */
    union {
        ucp_dt_reg_t              contig;
        struct {
            size_t                iov_offset;     /* Offset in the IOV item */
            size_t                iovcnt_offset;  /* The IOV item to start copy */
            size_t                iovcnt;         /* Number of IOV buffers */
            ucp_dt_reg_t          *dt_reg;        /* Pointer to IOV memh[iovcnt] */
        } iov;
        struct {
            void                  *state;
        } generic;
        struct {
            ucp_dt_reg_t          contig;     /* memh for contig space covering
                                                 all struct*/
            ucp_dt_reg_t          non_contig; /* indirect memh (umr) */
        } struct_dt;
    } dt;
} ucp_dt_state_t;

size_t ucp_dt_length(ucp_datatype_t datatype);
size_t ucp_dt_extent(ucp_datatype_t datatype);
size_t ucp_dt_low_bound(ucp_datatype_t datatype);

size_t ucp_dt_pack(ucp_worker_h worker, ucp_datatype_t datatype,
                   ucs_memory_type_t mem_type, void *dest, const void *src,
                   ucp_dt_state_t *state, size_t length);


ucs_status_t ucp_mem_type_pack(ucp_worker_h worker, void *dest,
                               const void *src, size_t length,
                               ucs_memory_type_t mem_type);


ucs_status_t ucp_mem_type_unpack(ucp_worker_h worker, void *buffer,
                                 const void *recv_data, size_t recv_length,
                                 ucs_memory_type_t mem_type);

#endif /* UCP_DT_H_ */

