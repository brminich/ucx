/**
 * Copyright (C) Mellanox Technologies Ltd. 2001-2015.  ALL RIGHTS RESERVED.
 *
 * See file LICENSE for terms.
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include "dt_struct.h"
#include "dt_contig.h"
#include "dt_iov.h"

#include <ucs/debug/assert.h>
#include <ucs/debug/memtrack.h>
#include <ucs/debug/assert.h>
#include <ucs/sys/math.h>
#include <uct/api/uct.h>
#include <ucp/core/ucp_ep.inl>
#include <ucp/core/ucp_mm.h>

#include <string.h>
#include <unistd.h>

#if ENABLE_STATS
static ucs_stats_class_t ucp_dt_struct_stats_class = {
    .name           = "dt_struct",
    .num_counters   = UCP_DT_STRUCT_STAT_LAST,
    .counter_names  = {
        [UCP_DT_STRUCT_STAT_CREATE]   = "create",
        [UCP_DT_STRUCT_STAT_IN_CACHE] = "reuse"
     }
};
#endif

ucs_status_t _struct_register_ep_rec(uct_ep_h ep, void *buf, ucp_dt_struct_t *s,
                                     uct_mem_h contig_memh, uct_mem_h* memh);

ucs_status_t _struct_register_rec(ucp_dt_struct_t *s,
                                  ucp_dt_struct_hash_value_t *val,
                                  void *buf);

static void _set_struct_attributes(ucp_dt_struct_t *s)
{
    size_t i, length = 0, iovs = 0;
    size_t depth = 0;
    size_t min_disp = SIZE_MAX;
    size_t max_disp = 0;
    size_t extent, lb;
    /* Use the middle of the 64-bit address space as the base address */
    size_t base_addr = 1L << ((sizeof(size_t) * 8) - 1);

    for (i = 0; i < s->desc_count; i++) {
        ucp_struct_dt_desc_t *dsc = &s->desc[i];
        switch (dsc->dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
            length += ucp_contig_dt_length(dsc->dt, 1);
            ++iovs;
            break;
        case UCP_DATATYPE_STRUCT:
            length += ucp_dt_struct_length(ucp_dt_struct(dsc->dt));
            iovs   += ucp_dt_struct(dsc->dt)->rep_count == 1 ?
                      ucp_dt_struct(dsc->dt)->uct_iov_count : 1;
            depth = ucs_max(depth, ucp_dt_struct_depth(ucp_dt_struct(dsc->dt)));
            break;
        default:
            /* Should not happen! */
            ucs_assertv(0, "wrong dt %ld", dsc->dt & UCP_DATATYPE_CLASS_MASK);
            break;

        }
        lb = base_addr + dsc->displ + ucp_dt_low_bound(dsc->dt);
        min_disp = ucs_min(min_disp, lb);
        /* NOTE:
         * It is not correct to calculate extent of a single repetition and
         * multiply by number of repetitions to get the final extent.
         * Example:
         * subdt1: |xxx|...|xxx|...|xxx|
         * subdt2:    |yy|..|yy|..|yy|
         * sing  : |-----|
         * singX3: |-----|-----|-----|
         * real  : |-------------------|
         *
         * Note, that for the stride, extent doesn't include the last padding
         * coming from the stride:
         * stride: |xxx|...|xxx|...|xxx|...|
         * extent: |xxxxxxxxxxxxxxxxxxx|
         * Thus the formua is: stride * (rep_count - 1) + payload
         */
        extent = dsc->extent * (s->rep_count - 1) + ucp_dt_extent(dsc->dt);
        max_disp = ucs_max(max_disp, lb + extent);
    }
    /* TODO: UMR will be created for repeated patterns, otherwise can unfold.
     * In case of nested umr just one iov would be enough. Need to distinguish
     * "leaf" and "nested" structs */

    s->uct_iov_count = iovs;
    s->step_len      = length;
    s->len           = length * s->rep_count;
    s->extent        = max_disp - min_disp;
    s->lb_displ      = (ptrdiff_t)min_disp - (ptrdiff_t)base_addr;
    s->depth = depth + 1;
}

/* Seek for the offset */
static ssize_t _elem_by_offset( const ucp_dt_struct_t *s, size_t offset,
                                size_t *rel_offset, size_t *rep_num)
{
    size_t toffs = 0, len = 0, i;

    /* First, find the sequential number of the repetition that holds this
     * offset
     */
    *rep_num = offset / s->step_len;
    if( !(*rep_num < s->rep_count) ) {
        /* Shouldn't happen */
        return -1;
    }
    toffs = (*rep_num) * s->step_len;

    for (i = 0; i < s->desc_count; i++) {
        ucp_struct_dt_desc_t *dsc = &s->desc[i];
        switch (dsc->dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
            len = ucp_contig_dt_length(dsc->dt, 1);
            break;
        case UCP_DATATYPE_STRUCT:
            len = ucp_dt_struct_length(ucp_dt_struct(dsc->dt));
            break;
        }
        if( (offset >= toffs) && (offset < toffs + len) ){
            *rel_offset = offset - toffs;
            return i;
        }
        toffs += len;
    }
    return -1;
}


static size_t _dte_pack( const ucp_dt_struct_t *s,
                         const void *inbuf, void *outbuf,
                         size_t out_offset_orig, size_t len)
{
    size_t out_offs = 0;
    size_t copy_len = 0;

    ssize_t elem_idx = -1;
    size_t elem_len = 0;
    size_t elem_offs_int = 0, elem_rep_num = 0;
    ptrdiff_t elem_offs = 0;
    ucp_dt_struct_t *sub_s;

    /* Seek for the offset */
    elem_idx = _elem_by_offset(s, out_offs, &elem_offs_int, &elem_rep_num);

    while( (0 < len) && elem_rep_num < s->rep_count){
        ucp_struct_dt_desc_t *dsc = &s->desc[elem_idx];
        elem_offs = dsc->displ + dsc->extent * elem_rep_num;
        switch (dsc->dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
            elem_len = ucp_contig_dt_length(dsc->dt, 1);
            copy_len = ucs_min(elem_len - elem_offs_int, len);
            memcpy(outbuf + out_offs,
                   (inbuf + elem_offs + elem_offs_int),
                   copy_len);
            break;
        case UCP_DATATYPE_STRUCT:
            sub_s = ucp_dt_struct(dsc->dt);
            copy_len = _dte_pack(sub_s, inbuf + elem_offs, outbuf + out_offs,
                                 elem_offs_int, len);
            break;
        }
        /* after the first iteration we will always be copying from the
         * beginning of each structural element
         */
        out_offs += copy_len;
        len -= copy_len;
        elem_offs_int = 0;
        elem_idx++;
        if(!(elem_idx < s->desc_count)) {
            elem_idx = 0;
            elem_rep_num++;
        }
    }

    /* Return processed length */
    return out_offs;
}

static size_t _dte_unpack(const ucp_dt_struct_t *s,
                          const void *inbuf, void *outbuf,
                         size_t in_offset_orig, size_t len)
{
    size_t in_offset = 0;
    size_t copy_len = 0;

    ssize_t elem_idx = -1;
    size_t elem_len = 0;
    size_t elem_offs_int = 0, elem_rep_num = 0;
    ptrdiff_t elem_offs = 0;
    ucp_dt_struct_t *sub_s;

    /* Seek for the offset */
    elem_idx = _elem_by_offset(s, in_offset_orig,
                               &elem_offs_int, &elem_rep_num);

    while( (0 < len) && elem_rep_num < s->rep_count){
        ucp_struct_dt_desc_t *dsc = &s->desc[elem_idx];
        elem_offs = dsc->displ + dsc->extent * elem_rep_num;
        switch (dsc->dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
            elem_len = ucp_contig_dt_length(dsc->dt, 1);
            copy_len = ucs_min(elem_len - elem_offs_int, len);
            memcpy((outbuf + elem_offs + elem_offs_int),
                   (inbuf + in_offset),
                   copy_len);
            break;
        case UCP_DATATYPE_STRUCT:
            sub_s = ucp_dt_struct(dsc->dt);
            copy_len = _dte_unpack(sub_s, inbuf + in_offset,
                                   outbuf + elem_offs, elem_offs_int,
                                   len);
            break;
        }
        /* after the first iteration we will always be copying from the
         * beginning of each structural element
         */
        in_offset += copy_len;
        len -= copy_len;
        elem_offs_int = 0;
        elem_idx++;
        if(!(elem_idx < s->desc_count)) {
            elem_idx = 0;
            elem_rep_num++;
        }
    }

    /* Return processed length */
    return in_offset;
}

ucs_status_t ucp_dt_create_struct(ucp_struct_dt_desc_t *desc_ptr,
                                  size_t desc_count, size_t rep_count,
                                  ucp_datatype_t *datatype_p)
{
    ucs_status_t status;
    ucp_dt_struct_t *dt;
    size_t i;

    /* Sanity check:
     * Structured datatype only supports UCP_DATATYPE_CONTIG and
     * UCP_DATATYPE_STRUCT as sub-datatypes
     */
    for(i = 0; i < desc_count; i++){
        switch (desc_ptr[i].dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_STRUCT:
            /* Additional check for unsupported configurations */
            if(1 != rep_count) {
                /* Currently we cannot repeat struct datatype
                 * as it requires re-registration of the UMR.
                 * TODO: fix this if needed in future */
                return UCS_ERR_NOT_IMPLEMENTED;
            }
        case UCP_DATATYPE_CONTIG:
            /* OK */
            break;
        case UCP_DATATYPE_IOV:
        case UCP_DATATYPE_GENERIC:
            /* Not supported */
            return UCS_ERR_NOT_IMPLEMENTED;
        }
    }

    dt = ucs_calloc(1, sizeof(*dt), "dt_struct");
    if (dt == NULL) {
        return UCS_ERR_NO_MEMORY;
    }

    dt->desc = ucs_calloc(desc_count, sizeof(*dt->desc), "dt_desc");
    if (dt->desc == NULL) {
        ucs_free(dt);
        return UCS_ERR_NO_MEMORY;
    }

    kh_init_inplace(dt_struct, &dt->hash);

    memcpy(dt->desc, desc_ptr, sizeof(*desc_ptr) * desc_count);
    dt->desc_count = desc_count;
    dt->rep_count = rep_count;

    _set_struct_attributes(dt);
    *datatype_p = ((uintptr_t)dt) | UCP_DATATYPE_STRUCT;



    status = UCS_STATS_NODE_ALLOC(&dt->stats,
                                  &ucp_dt_struct_stats_class,
                                  ucs_stats_get_root(), "%p-%d-%d",
                                  dt, desc_count, rep_count);
    if (status != UCS_OK) {
        ucs_error("Can't allocate stats: %s", ucs_status_string(status));
        return status;
    }

    ucs_info("Created struct dt %p, len %ld (step %ld), depth %ld, uct_iovs %ld, rep count %ld",
             dt, dt->len, dt->step_len, dt->depth, dt->uct_iov_count, dt->rep_count);

    return UCS_OK;
}

void ucp_dt_destroy_struct(ucp_datatype_t datatype_p)
{
    ucp_dt_struct_t *dt = ucp_dt_struct(datatype_p);
    ucp_dt_struct_hash_value_t val;
    ucs_status_t status;

    ucs_info("Destroy struct dt %p, len %ld (step %ld), depth %ld, uct_iovs %ld",
             dt, dt->len, dt->step_len, dt->depth, dt->uct_iov_count);

    kh_foreach_value(&dt->hash, val, {
        uct_md_h md = val.ucp_ctx->tl_mds[val.md_idx].md;
        ucs_info("struct dt %p, dereg NC memh %p on md %p",
                 dt, val.noncontig.memh, md);
        uct_md_mem_dereg_nc(md, val.noncontig.memh[0]);
        status = ucp_mem_rereg_mds(val.ucp_ctx, 0, NULL, 0, 0, NULL,
                                   UCS_MEMORY_TYPE_HOST, NULL,
                                   val.contig.memh, &val.contig.md_map);
        assert(UCS_OK == status);
    })
    kh_destroy_inplace(dt_struct, &dt->hash);
    ucs_free(dt->desc);
    UCS_STATS_NODE_FREE(dt->stats);
    ucs_free(dt);
}

void ucp_dt_struct_gather(void *dest, const void *src, ucp_datatype_t dt,
                          size_t length, size_t offset)
{
    size_t processed_len;
    ucp_dt_struct_t *s = ucp_dt_struct(dt);
    /* TODO: enable using "state" to make it more efficient.
     * Right now it always performs the "seek" operation which is
     * inefficient
     */
     processed_len = _dte_pack(s, src, dest, offset, length);

     /* We assume that the sane length was provided */
     ucs_assert(processed_len == length);
}

size_t ucp_dt_struct_scatter(void *dst, ucp_datatype_t dt,
                             const void *src, size_t length, size_t offset)
{
    size_t processed_len;
    ucp_dt_struct_t *s = ucp_dt_struct(dt);
    /* TODO: enable using "state" to make it more efficient.
     * Right now it always performs the "seek" operation which is
     * inefficient
     */
     processed_len = _dte_unpack(s, src, dst, offset, length);

     /* We assume that the sane length was provided */
     ucs_assert(processed_len == length);
     return processed_len;
}

/* Dealing with UCT */

inline static void _to_cache(ucp_dt_struct_t *s, void *ptr,
                             ucp_dt_struct_hash_value_t *val)
{
    uct_md_h md = val->ucp_ctx->tl_mds[val->md_idx].md;
    khiter_t k;
    int ret;

    k = kh_put(dt_struct, &s->hash, (uint64_t)ptr, &ret);
    ucs_assert_always((ret == 1) || (ret == 2));
    /* TODO: check ret */
    kh_value(&s->hash, k) = *val;

    ucs_info("dt %p adding to cache (buf %p md %p memh %p)", s, ptr, md,
             val->noncontig.memh);
}

#if 0
static uct_iov_t* _fill_uct_iov_rec(uct_ep_h ep, void *buf, ucp_dt_struct_t *s,
                                    uct_mem_h contig_memh, uct_iov_t *iovs)
{
    uct_iov_t *iov = iovs;
    ucp_dt_struct_t *s_in;
    ucs_status_t status;
    void *ptr;
    int i;

    for (i = 0; i < s->desc_count; i++, iov++) {
        ptr = UCS_PTR_BYTE_OFFSET(buf, s->desc[i].displ);

        if (UCP_DT_IS_STRUCT(s->desc[i].dt)) {
            s_in = ucp_dt_struct(s->desc[i].dt);
            if (s_in->rep_count == 1) {
                iov = _fill_uct_iov_rec(ep, ptr, s_in, contig_memh, iov);
            } else {
                iov->buffer = ptr;
                iov->length = s_in->len;
                iov->stride = s->desc[i].extent;
                status      = _struct_register_ep_rec(ep, ptr, s_in,
                                                      contig_memh, &iov->memh);
                ucs_assert_always(status == UCS_OK);
            }
        } else {
            /* On the leaf level, all datatypes are contig */
            iov->buffer = ptr;
            iov->length = ucp_contig_dt_length(s->desc[i].dt, 1);
            iov->stride = s->desc[i].extent;
            iov->memh   = contig_memh;
        }
    }

    return iov;
}


ucs_status_t _struct_register_ep_rec(uct_ep_h ep, void *buf, ucp_dt_struct_t *s,
                                     uct_mem_h contig_memh, uct_mem_h* memh)
{
    size_t iov_cnt  = s->uct_iov_count;
    uct_iov_t *iovs = ucs_calloc(iov_cnt, sizeof(*iovs), "umr_iovs");
    ucs_status_t status;
    uct_md_h md_p;
    uct_completion_t comp;

    _fill_uct_iov_rec(ep, buf, s, contig_memh, iovs);

    status = uct_ep_mem_reg_nc(ep, iovs, iov_cnt, s->rep_count,
                               &md_p, /* remove me */
                               &memh[0], /* revise */
                               &comp);
    if (status != UCS_OK) {
        ucs_error("Failed to register NC memh: %s", ucs_status_string(status));
        return status;
    }

    /* TODO: wait for completion (now uct_ep_mem_reg_nc is blocking) */

    ucs_free(iovs); /* optimize */

    return UCS_OK;
}


ucs_status_t ucp_dt_struct_register_ep(ucp_ep_h ep, ucp_lane_index_t lane,
                                       void *buf, ucp_datatype_t dt, uct_mem_h
                                       contig_memh, uct_mem_h* memh,
                                       ucp_md_map_t *md_map_p)
{
    ucp_dt_struct_t *s    = ucp_dt_struct(dt);
    uct_ep_h uct_ep       = ep->uct_eps[lane];
    ucp_md_index_t md_idx = ucp_ep_md_index(ep, lane);
    uct_md_h md           = ep->worker->context->tl_mds[md_idx].md;
    ucs_status_t status;

    ucs_assert_always(UCP_DT_IS_STRUCT(dt));

    ucs_info("Register struct on ep %ld, len %ld", dt, s->len);

    status = _struct_register_ep_rec(uct_ep, buf, s, contig_memh, memh);
    if (status == UCS_OK) {
        *md_map_p = UCS_BIT(md_idx);
        _to_cache(s, buf, md, memh[0]);

    }

    return status;
}
#endif

static uct_iov_t* _fill_md_uct_iov_rec(ucp_dt_struct_t *s,
                                       ucp_dt_struct_hash_value_t *val,
                                       void *buf,
                                       uct_iov_t *iovs)
{
    uct_iov_t *iov = iovs;
    ucp_dt_struct_t *s_in;
    ucs_status_t status;
    void *ptr, *eptr;
    ucp_md_map_t md_map = 0;
    int i;

    for (i = 0; i < s->desc_count; i++, iov++) {
        ptr = UCS_PTR_BYTE_OFFSET(buf, s->desc[i].displ);
        eptr = UCS_PTR_BYTE_OFFSET(ptr, ucp_dt_low_bound(s->desc[i].dt));
        if (UCP_DT_IS_STRUCT(s->desc[i].dt)) {
            s_in = ucp_dt_struct(s->desc[i].dt);
            if (s_in->rep_count == 1) {
                iov = _fill_md_uct_iov_rec(s_in, val, ptr, iov);
            } else {
                /* calculate effective offset */
                iov->buffer = eptr;
                iov->length = s_in->len;
                iov->stride = s->desc[i].extent;
                status = ucp_dt_struct_register(val->ucp_ctx, val->md_idx, ptr,
                                                s->desc[i].dt, &iov->memh,
                                                &md_map);
                ucs_assert_always(status == UCS_OK);
            }
        } else {
            /* On the leaf level, all datatypes are contig */
            iov->buffer = ptr;
            iov->length = ucp_contig_dt_length(s->desc[i].dt, 1);
            iov->stride = s->desc[i].extent;
            iov->memh   = val->contig.memh[0];
        }
    }

    return iov;
}

ucs_status_t _struct_register_rec(ucp_dt_struct_t *s,
                                  ucp_dt_struct_hash_value_t *val,
                                  void *buf)
{
    uct_md_h md = val->ucp_ctx->tl_mds[val->md_idx].md;
    size_t iov_cnt  = s->uct_iov_count;
    uct_iov_t *iovs = ucs_calloc(iov_cnt, sizeof(*iovs), "umr_iovs");
    ucs_status_t status;

    _fill_md_uct_iov_rec(s, val, buf, iovs);

    status = uct_md_mem_reg_nc(md, iovs, iov_cnt, s->rep_count, &val->noncontig.memh[0]);
    if (status != UCS_OK) {
        ucs_error("Failed to register NC memh: %s", ucs_status_string(status));
        return status;
    }

    /* TODO: wait for completion (now uct_ep_mem_reg_nc is blocking) */

    ucs_free(iovs); /* optimize */

    return UCS_OK;
}

ucs_status_t ucp_dt_struct_register(ucp_context_t *context, ucp_md_index_t md_idx,
                                    void *buf, ucp_datatype_t dt,
                                    uct_mem_h* memh,
                                    ucp_md_map_t *md_map_p)
{
    ucp_dt_struct_t *s = ucp_dt_struct(dt);
    ucs_status_t status;
    ucp_dt_struct_hash_value_t val;

    ucs_assert_always(UCP_DT_IS_STRUCT(dt));

    /* register contig memory block covering the whole struct
     * This will ensure that the memory wil not be invalidated
     */
    val.ucp_ctx = context;
    val.md_idx = md_idx;
    val.contig.md_map = 0;
    status = ucp_mem_rereg_mds(val.ucp_ctx, UCS_BIT(val.md_idx),
                               buf + s->lb_displ,
                               s->extent,
                               UCT_MD_MEM_ACCESS_ALL, NULL,
                               UCS_MEMORY_TYPE_HOST, NULL,
                               val.contig.memh,
                               &val.contig.md_map);

#if 0
    printf("STRUCT reg: addr=%p, datatype=%p\n", buf, s);
#endif

    ucs_info("Register struct on md, dt %ld, len %ld", dt, s->len);

    status = _struct_register_rec(s, &val, buf);
    if (status == UCS_OK) {
        //*md_map_p = UCS_BIT(md_idx);
        _to_cache(s, buf, &val);

    }
    *memh = val.noncontig.memh[0];

    return status;
}
