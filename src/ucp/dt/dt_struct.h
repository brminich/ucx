/**
 * Copyright (C) Mellanox Technologies Ltd. 2001-2015.  ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */

#ifndef DT_STRUCT_H_
#define DT_STRUCT_H_

#include <ucp/core/ucp_context.h>
#include <ucp/api/ucp.h>
#include <ucs/datastruct/khash.h>
#include <uct/api/uct.h>
#include <ucp/core/ucp_types.h>
#include <ucs/stats/stats.h>
#include "dt_common.h"

typedef struct ucp_dt_struct_hash_value {
    ucp_context_t *ucp_ctx;
    ucp_md_index_t md_idx;
    ucp_dt_reg_t contig;
    ucp_dt_reg_t noncontig;
} ucp_dt_struct_hash_value_t;

KHASH_MAP_INIT_INT64(dt_struct, ucp_dt_struct_hash_value_t)
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

enum {
    UCP_DT_STRUCT_STAT_CREATE,
    UCP_DT_STRUCT_STAT_IN_CACHE,
    UCP_DT_STRUCT_STAT_LAST
};

/**
 * Structured datatype structure.
 */
typedef struct ucp_dt_struct {
    ucp_struct_dt_desc_t *desc;
    size_t len, step_len, depth;
    size_t desc_count;
    size_t rep_count;
    size_t uct_iov_count; /* total count of needed UCT iovs for unfolded struct */
    size_t extent; /* total contig space covering the whole type */
    ptrdiff_t lb_displ; /* the lowest displacement from which extent is effective */
    khash_t(dt_struct) hash;
    UCS_STATS_NODE_DECLARE(stats);
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
 * Get the total length of the structured datatype
 */
static inline size_t ucp_dt_struct_extent(const ucp_dt_struct_t *s)
{
    return s->extent;
}

/**
 * Get the total length of the structured datatype
 */
static inline size_t ucp_dt_struct_lb(const ucp_dt_struct_t *s)
{
    return s->lb_displ;
}

/**
 * Get the max depth of the struct
 */
static inline size_t ucp_dt_struct_depth(const ucp_dt_struct_t *s)
{
    return s->depth;
}

static UCS_F_ALWAYS_INLINE uct_mem_h ucp_dt_struct_in_cache(ucp_dt_struct_t *s,
                                                            void *ptr)
{
    khiter_t k;
    k = kh_get(dt_struct, &s->hash, (uint64_t)ptr);

#if 0
    printf("STRUCT rcache req: addr=%p, datatype=%p\n", ptr, s);
#endif
    return (k == kh_end(&s->hash)) ?
                NULL : kh_value(&s->hash, k).noncontig.memh[0];
}


ucs_status_t ucp_dt_create_struct(ucp_struct_dt_desc_t *desc_ptr,
                                  size_t desc_count, size_t rep_count,
                                  ucp_datatype_t *datatype_p);
void ucp_dt_destroy_struct(ucp_datatype_t datatype_p);

void ucp_dt_struct_gather(void *dest, const void *src, ucp_datatype_t dt,
                          size_t length, size_t offset);

size_t ucp_dt_struct_scatter(void *dst, ucp_datatype_t dt, const void *src,
                          size_t length, size_t offset);

ucs_status_t ucp_dt_struct_register_ep(ucp_ep_h ep, ucp_lane_index_t lane,
                                       void *buf, ucp_datatype_t dt, uct_mem_h
                                       contig_memh, uct_mem_h* memh,
                                       ucp_md_map_t *md_map_p);

ucs_status_t ucp_dt_struct_register(ucp_context_t *context, ucp_md_index_t md_idx,
                                    void *buf, ucp_datatype_t dt,
                                    uct_mem_h* memh,
                                    ucp_md_map_t *md_map_p);

#endif // DT_STRUCT_H
