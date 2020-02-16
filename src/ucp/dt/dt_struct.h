/**
 * Copyright (C) Mellanox Technologies Ltd. 2001-2015.  ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */

#ifndef DT_STRUCT_H_
#define DT_STRUCT_H_

#include <ucp/api/ucp.h>
#include <ucs/datastruct/khash.h>
#include <src/uct/api/uct.h>

KHASH_MAP_INIT_INT64(dt_struct, uct_mem_h)
/*
int main() {
    int ret, is_missing;
    khiter_t k;
    khash_t(32) *h = kh_init(32);
    k = kh_put(32, h, 5, &ret);
    kh_value(h, k) = 10;
    k = kh_get(32, h, 10);
    is_missing = (k == kh_end(h));
    k = kh_get(32, h, 5);
    kh_del(32, h, k);
    for (k = kh_begin(h); k != kh_end(h); ++k)
        if (kh_exist(h, k)) kh_value(h, k) = 1;
    kh_destroy(32, h);
    return 0;
}
*/

/**
 * Structured datatype structure.
 */
typedef struct ucp_dt_struct {
    ucp_struct_dt_desc_t *desc;
    size_t len, step_len, depth;
    size_t desc_count;
    size_t rep_count;
    khash_t(dt_struct) hash;
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
