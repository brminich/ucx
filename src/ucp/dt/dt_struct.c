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

#include <string.h>
#include <unistd.h>

static void _set_length(ucp_dt_struct_t *s)
{
    size_t i, length = 0;

    for (i = 0; i < s->desc_count; i++) {
        ucp_struct_dt_desc_t *dsc = &s->desc_ptr[i];
        switch (dsc->dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
            length += ucp_contig_dt_length(dsc->dt, 1);
            break;
        case UCP_DATATYPE_STRUCT:
            length += ucp_dt_struct_length(ucp_dt_struct(dsc->dt));
        case UCP_DATATYPE_IOV:
        case UCP_DATATYPE_GENERIC:
            /* These types are not supported in the struct datatype */
        default:
            /* Should not happen! */
            ucs_assert(0);
            break;

        }
    }
    s->step_len = length;
    s->len = length * s->rep_count;
}

static void _set_depth(ucp_dt_struct_t *s)
{
    size_t i, depth = 0;

    for (i = 0; i < s->desc_count; i++) {
        ucp_struct_dt_desc_t *dsc = &s->desc_ptr[i];
        switch (dsc->dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
            depth = ucs_max(depth, 1);
            break;
        case UCP_DATATYPE_STRUCT:
            depth = ucs_max(depth, ucp_dt_struct_depth(ucp_dt_struct(dsc->dt)));
            break;
        case UCP_DATATYPE_IOV:
        case UCP_DATATYPE_GENERIC:
            /* These types are not supported in the struct datatype */
        default:
            /* Should not happen! */
            ucs_assert(0);
            break;

        }
    }
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
        ucp_struct_dt_desc_t *dsc = &s->desc_ptr[i];
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
    ssize_t eidx = -1;
    size_t elem_len = 0, copy_len = 0;
    size_t out_offset = out_offset_orig;
    size_t out_offset_rel = 0, rep_num = 0;
    ptrdiff_t in_offset = 0;
    ucp_dt_struct_t *sub_s;

    /* Seek for the offset */
    eidx = _elem_by_offset(s, out_offset, &out_offset_rel, &rep_num);

    while( (0 < len) && rep_num < s->rep_count){
        ucp_struct_dt_desc_t *dsc = &s->desc_ptr[eidx];
        in_offset = dsc->displ + dsc->extent * rep_num;
        switch (dsc->dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
            elem_len = ucp_contig_dt_length(dsc->dt, 1);
            copy_len = ucs_min(elem_len - out_offset_rel, len);
            memcpy(outbuf + out_offset, inbuf + in_offset, copy_len);
            break;
        case UCP_DATATYPE_STRUCT:
            sub_s = ucp_dt_struct(dsc->dt);
            copy_len = _dte_pack(sub_s, inbuf + in_offset, outbuf + out_offset,
                                 out_offset_rel, len);
            break;
        }
        /* after the first iteration we will always be copying from the
         * beginning of each structural element
         */
        out_offset += copy_len;
        len -= copy_len;
        out_offset_rel = 0;
        eidx++;
        if(!(eidx < s->desc_count)) {
            eidx = 0;
            rep_num++;
        }
    }

    /* Return processed length */
    return out_offset - out_offset_orig;
}

static size_t _dte_unpack(const ucp_dt_struct_t *s,
                          const void *inbuf, void *outbuf,
                         size_t in_offset_orig, size_t len)
{
    ssize_t eidx = -1;
    size_t elem_len = 0, copy_len = 0;
    size_t in_offset = in_offset_orig;
    size_t in_offset_rel = 0, rep_num = 0;
    ptrdiff_t out_offset = 0;
    ucp_dt_struct_t *sub_s;

    /* Seek for the offset */
    eidx = _elem_by_offset(s, in_offset, &in_offset_rel, &rep_num);

    while( (0 < len) && rep_num < s->rep_count){
        ucp_struct_dt_desc_t *dsc = &s->desc_ptr[eidx];
        out_offset = dsc->displ + dsc->extent * rep_num;
        switch (dsc->dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
            elem_len = ucp_contig_dt_length(dsc->dt, 1);
            copy_len = ucs_min(elem_len - in_offset_rel, len);
            memcpy(outbuf + out_offset, inbuf + in_offset, copy_len);
            break;
        case UCP_DATATYPE_STRUCT:
            sub_s = ucp_dt_struct(dsc->dt);
            copy_len = _dte_unpack(sub_s, inbuf + in_offset, outbuf + out_offset,
                      in_offset_rel, len);
            break;
        }
        /* after the first iteration we will always be copying from the
         * beginning of each structural element
         */
        in_offset += copy_len;
        len -= copy_len;
        in_offset_rel = 0;
        eidx++;
        if(!(eidx < s->desc_count)) {
            eidx = 0;
            rep_num++;
        }
    }

    /* Return processed length */
    return (in_offset - in_offset_orig);
}

ucs_status_t ucp_dt_create_struct(ucp_struct_dt_desc_t *desc_ptr,
                                  size_t desc_count, size_t rep_count,
                                  ucp_datatype_t *datatype_p)
{
    ucp_dt_struct_t *dt;
    int ret;
    size_t i;

    for(i=0; i < desc_count; i++){
        switch (desc_ptr[i].dt & UCP_DATATYPE_CLASS_MASK) {
        case UCP_DATATYPE_CONTIG:
        case UCP_DATATYPE_STRUCT:
            /* OK */
            break;
        case UCP_DATATYPE_IOV:
        case UCP_DATATYPE_GENERIC:
            /* Not supported */
            return UCS_ERR_NOT_IMPLEMENTED;
        }
    }

    /* Sanity check:
     * Structured datatype only supports UCP_DATATYPE_CONTIG and
     * UCP_DATATYPE_STRUCT as sub-datatypes
     */

    ret = ucs_posix_memalign((void **)&dt,
                             ucs_max(sizeof(void *), UCS_BIT(UCP_DATATYPE_SHIFT)),
                             sizeof(*dt), "struct_dt");
    if (ret != 0) {
        return UCS_ERR_NO_MEMORY;
    }

    ret = ucs_posix_memalign((void **)&dt->desc_ptr, sizeof(*dt->desc_ptr),
                             sizeof(*dt->desc_ptr) * dt->desc_count,
                             "ucp_dt_struct_t");
    if (ret != 0) {
        ucs_free(dt);
        return UCS_ERR_NO_MEMORY;
    }
    memcpy(dt->desc_ptr, desc_ptr, sizeof(*desc_ptr) * desc_count);
    dt->desc_count = desc_count;
    dt->rep_count = rep_count;
    _set_length(dt);
    _set_depth(dt);
    *datatype_p = ((uintptr_t)dt) | UCP_DATATYPE_STRUCT;
    return UCS_OK;
}

void ucp_dt_destroy_struct(ucp_datatype_t datatype_p)
{
    ucp_dt_struct_t *dt;
    dt = ucp_dt_struct(datatype_p);
    ucs_free(dt->desc_ptr);
    ucs_free(dt);
}

void ucp_dt_struct_gather(void *dest, const void *src, ucp_datatype_t dt,
                          size_t length, size_t offset, void *state)
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
                             const void *src, size_t length, size_t offset,
                             void *state)
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
