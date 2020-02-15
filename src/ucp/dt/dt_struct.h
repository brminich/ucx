/**
 * Copyright (C) Mellanox Technologies Ltd. 2001-2015.  ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */

#ifndef DT_STRUCT_H_
#define DT_STRUCT_H_

#include <ucp/api/ucp.h>

/**
 * Structured datatype structure.
 */
typedef struct ucp_dt_struct {
    ucp_struct_dt_desc_t *desc_ptr;
    size_t len, step_len, depth;
    size_t desc_count;
    size_t rep_count;
} ucp_dt_struct_t;

static inline ucp_dt_struct_t* ucp_dt_struct(ucp_datatype_t datatype)
{
    return (ucp_dt_struct_t*)(void*)(datatype & ~UCP_DATATYPE_CLASS_MASK);
}
#define UCP_DT_IS_STRUCT(_datatype) \
          (((_datatype) & UCP_DATATYPE_CLASS_MASK) == UCP_DATATYPE_STRUCT)


/**
 * Get the total length of the structured datatype
 */
static inline size_t ucp_dt_struct_length(const ucp_dt_struct_t *s)
{
    return s->len;
}

/**
 * Get the max depth of the struct
 */
static inline size_t ucp_dt_struct_depth(const ucp_dt_struct_t *s)
{
    return s->depth;
}


ucs_status_t ucp_dt_create_struct(ucp_struct_dt_desc_t *desc_ptr,
                                  size_t desc_count, size_t rep_count,
                                  ucp_datatype_t *datatype_p);
void ucp_dt_destroy_struct(ucp_datatype_t datatype_p);

void ucp_dt_struct_gather(void *dest, const void *src, ucp_datatype_t dt,
                          size_t length, size_t offset, void *state);

size_t ucp_dt_struct_scatter(void *dst, ucp_datatype_t dt, const void *src,
                          size_t length, size_t offset, void *state);

#endif // DT_STRUCT_H
