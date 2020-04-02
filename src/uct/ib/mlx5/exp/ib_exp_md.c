/**
* Copyright (C) Mellanox Technologies Ltd. 2019.  ALL RIGHTS RESERVED.
*
* See file LICENSE for terms.
*/

#include <uct/ib/mlx5/ib_mlx5.h>

#include <ucs/arch/bitops.h>
#include <ucs/profile/profile.h>

typedef struct uct_ib_umr uct_ib_umr_t;

typedef struct {
    struct ibv_mr              *atomic_mr;
    int                        mr_num;
    struct ibv_mr              *mrs[];
} uct_ib_mlx5_ksm_data_t;

typedef struct uct_ib_mlx5_mem {
    uct_ib_mem_t               super;
    struct ibv_mr              *mr;
#if HAVE_EXP_UMR
    union {
        struct ibv_mr          *atomic_mr;
        uct_ib_mlx5_ksm_data_t *ksm_data;
    };
    size_t                     umr_depth;
    uct_ib_umr_t               *umr;
#endif
} uct_ib_mlx5_mem_t;

struct uct_ib_umr {
    unsigned               depth;
    int                    is_inline;
    uct_ib_mlx5_mem_t      memh;        /* memh for indirect mr*/
    uct_ib_mlx5_mem_t      *contig_memh; /* memh for contig mr*/
    struct ibv_exp_send_wr wr;
    size_t                 repeat_count; /* 0 is not allowed; if 1 it is UMR
                                            list, otherwise repeated block */
    size_t                 iov_count;
    uint64_t               base_addr;

    uct_completion_t       comp;   /* completion routine */
    //ep_post_dereg_f        dereg_f; /* endpoint WR posting function pointer */
    uct_ep_t               *tl_ep;  /* registering endpoint - for cleanup */
};

static ucs_status_t uct_ib_mlx5_reg_key(uct_ib_md_t *md, void *address,
                                        size_t length, uint64_t access,
                                        uct_ib_mem_t *ib_memh)
{
    uct_ib_mlx5_mem_t *memh = ucs_derived_of(ib_memh, uct_ib_mlx5_mem_t);
    ucs_status_t status;

    status = uct_ib_reg_mr(md->pd, address, length, access, &memh->mr);
    if (status != UCS_OK) {
        return status;
    }

    uct_ib_memh_init_from_mr(&memh->super, memh->mr);
    return UCS_OK;
}

static ucs_status_t uct_ib_mlx5_dereg_key(uct_ib_md_t *md, uct_ib_mem_t *ib_memh)
{
    uct_ib_mlx5_mem_t *memh = ucs_derived_of(ib_memh, uct_ib_mlx5_mem_t);

    if (memh->super.flags & UCT_IB_MEM_FLAG_NC_BASE_MR) {
        memh->super.flags |= UCT_IB_MEM_FLAG_NEED_DEREG;
        return UCS_OK;
    }

    return uct_ib_dereg_mr(memh->mr);
}

static ucs_status_t
uct_ib_mlx5_mem_prefetch(uct_ib_md_t *md, uct_ib_mem_t *ib_memh, void *addr,
                         size_t length)
{
#if HAVE_DECL_IBV_EXP_PREFETCH_MR
    uct_ib_mlx5_mem_t *memh = ucs_derived_of(ib_memh, uct_ib_mlx5_mem_t);
    struct ibv_exp_prefetch_attr attr = {};
    int ret;

    if (!(memh->super.flags & UCT_IB_MEM_FLAG_ODP)) {
        return UCS_OK;
    }

    ucs_debug("memh %p prefetch %p length %zu", memh, addr, length);

    attr.flags     = IBV_EXP_PREFETCH_WRITE_ACCESS;
    attr.addr      = addr;
    attr.length    = length;

    ret = UCS_PROFILE_CALL(ibv_exp_prefetch_mr, memh->mr, &attr);
    if (ret) {
        ucs_error("ibv_exp_prefetch_mr(addr=%p length=%zu) returned %d: %m",
                  addr, length, ret);
        return UCS_ERR_IO_ERROR;
    }
#endif
    return UCS_OK;
}

static inline ucs_status_t
uct_ib_md_calc_required_klms(uct_ib_mlx5_md_t *md, const uct_iov_t *iov,
                             size_t iovcnt, unsigned *klms_needed,
                             unsigned *depth)
{
    struct ibv_exp_device_attr *dev_attr = &md->super.dev.dev_attr;
    unsigned max_depth = IBV_DEVICE_UMR_CAPS(dev_attr, max_umr_recursion_depth);
    unsigned iov_depth, umr_depth, iov_idx;

    if (iovcnt > IBV_DEVICE_UMR_CAPS(dev_attr, max_klm_list_size)) {
        return UCS_ERR_UNSUPPORTED;
    }

    for (iov_idx = 0, umr_depth = 0; iov_idx < iovcnt; iov_idx++) {
        uct_mem_h iov_memh = iov[iov_idx].memh;
        if (ucs_unlikely(!iov_memh)) {
            ucs_error("Invalid memh in UCT iov");
            return UCS_ERR_INVALID_PARAM;
        }

        /* Check if recursion depth limit is exceeded */
        iov_depth = ((uct_ib_mlx5_mem_t*)(iov_memh))->umr_depth;
        if (iov_depth > umr_depth) {
            if (iov_depth >= max_depth) {
                ucs_error("iov depth (%d) is bigger than max (%d)",
                          iov_depth, umr_depth);
                return UCS_ERR_UNSUPPORTED;
            }
            umr_depth = iov_depth;
        }
    }


    *klms_needed = iovcnt;
    *depth = umr_depth + 1;
    return UCS_OK;
}

static ucs_status_t uct_ib_mlx5_exp_md_umr_qp_create(uct_ib_mlx5_md_t *md)
{
#if HAVE_EXP_UMR
    struct ibv_exp_qp_init_attr qp_init_attr;
    struct ibv_qp_attr qp_attr;
    uint8_t port_num;
    int ret;
    uct_ib_device_t *ibdev;
    struct ibv_port_attr *port_attr;

    ibdev = &md->super.dev;

    if (!(ibdev->dev_attr.exp_device_cap_flags & IBV_EXP_DEVICE_UMR) ||
        !md->super.config.enable_indirect_atomic) {
        return UCS_ERR_UNSUPPORTED;
    }

    /* TODO: fix port selection. It looks like active port should be used */
    port_num = ibdev->first_port;
    port_attr = uct_ib_device_port_attr(ibdev, port_num);

    memset(&qp_init_attr, 0, sizeof(qp_init_attr));

    md->umr_cq = ibv_create_cq(ibdev->ibv_context, 1, NULL, NULL, 0);
    if (md->umr_cq == NULL) {
        ucs_error("failed to create UMR CQ: %m");
        goto err;
    }

    md->super.config.max_inline_klm_list =
        ucs_min(md->super.config.max_inline_klm_list,
                ibdev->dev_attr.umr_caps.max_send_wqe_inline_klms);

    qp_init_attr.qp_type             = IBV_QPT_RC;
    qp_init_attr.send_cq             = md->umr_cq;
    qp_init_attr.recv_cq             = md->umr_cq;
    qp_init_attr.cap.max_inline_data = 0;
    qp_init_attr.cap.max_recv_sge    = 1;
    qp_init_attr.cap.max_send_sge    = 1;
    qp_init_attr.srq                 = NULL;
    qp_init_attr.cap.max_recv_wr     = 16;
    qp_init_attr.cap.max_send_wr     = 16;
    qp_init_attr.pd                  = md->super.pd;
    qp_init_attr.comp_mask           = IBV_EXP_QP_INIT_ATTR_PD|IBV_EXP_QP_INIT_ATTR_MAX_INL_KLMS;
    qp_init_attr.max_inl_recv        = 0;
    qp_init_attr.max_inl_send_klms   = md->super.config.max_inline_klm_list;

#if HAVE_IBV_EXP_QP_CREATE_UMR
    qp_init_attr.comp_mask          |= IBV_EXP_QP_INIT_ATTR_CREATE_FLAGS;
    qp_init_attr.exp_create_flags    = IBV_EXP_QP_CREATE_UMR;
#endif

    md->umr_qp = ibv_exp_create_qp(ibdev->ibv_context, &qp_init_attr);
    if (md->umr_qp == NULL) {
        ucs_error("failed to create UMR QP: %m");
        goto err_destroy_cq;
    }

    memset(&qp_attr, 0, sizeof(qp_attr));

    /* Modify QP to INIT state */
    qp_attr.qp_state                 = IBV_QPS_INIT;
    qp_attr.pkey_index               = 0;
    qp_attr.port_num                 = port_num;
    qp_attr.qp_access_flags          = UCT_IB_MEM_ACCESS_FLAGS;
    ret = ibv_modify_qp(md->umr_qp, &qp_attr,
                        IBV_QP_STATE | IBV_QP_PKEY_INDEX | IBV_QP_PORT | IBV_QP_ACCESS_FLAGS);
    if (ret) {
        ucs_error("Failed to modify UMR QP to INIT: %m");
        goto err_destroy_qp;
    }

    /* Modify to RTR */
    qp_attr.qp_state                 = IBV_QPS_RTR;
    qp_attr.dest_qp_num              = md->umr_qp->qp_num;

    memset(&qp_attr.ah_attr, 0, sizeof(qp_attr.ah_attr));
    qp_attr.ah_attr.port_num         = port_num;
    qp_attr.ah_attr.dlid             = port_attr->lid;
    qp_attr.ah_attr.is_global        = 1;
    if (uct_ib_device_query_gid(ibdev, port_num, UCT_IB_MD_DEFAULT_GID_INDEX,
                                &qp_attr.ah_attr.grh.dgid) != UCS_OK) {
        goto err_destroy_qp;
    }

    qp_attr.rq_psn                   = 0;
    qp_attr.path_mtu                 = IBV_MTU_512;
    qp_attr.min_rnr_timer            = 7;
    qp_attr.max_dest_rd_atomic       = 1;
    ret = ibv_modify_qp(md->umr_qp, &qp_attr,
                        IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU | IBV_QP_DEST_QPN |
                        IBV_QP_RQ_PSN | IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_MIN_RNR_TIMER);
    if (ret) {
        ucs_error("Failed to modify UMR QP to RTR: %m");
        goto err_destroy_qp;
    }

    /* Modify to RTS */
    qp_attr.qp_state                 = IBV_QPS_RTS;
    qp_attr.sq_psn                   = 0;
    qp_attr.timeout                  = 7;
    qp_attr.rnr_retry                = 7;
    qp_attr.retry_cnt                = 7;
    qp_attr.max_rd_atomic            = 1;
    ret = ibv_modify_qp(md->umr_qp, &qp_attr,
                        IBV_QP_STATE | IBV_QP_TIMEOUT | IBV_QP_RETRY_CNT |
                        IBV_QP_RETRY_CNT | IBV_QP_RNR_RETRY | IBV_QP_SQ_PSN |
                        IBV_QP_MAX_QP_RD_ATOMIC);
    if (ret) {
        ucs_error("Failed to modify UMR QP to RTS: %m");
        goto err_destroy_qp;
    }

    ucs_debug("initialized UMR QP 0x%x, max_inline_klm_list %u",
              md->umr_qp->qp_num, md->super.config.max_inline_klm_list);
    return UCS_OK;

err_destroy_qp:
    uct_ib_destroy_qp(md->umr_qp);
err_destroy_cq:
    ibv_destroy_cq(md->umr_cq);
err:
    return UCS_ERR_IO_ERROR;
#else
    return UCS_ERR_UNSUPPORTED;
#endif
}

#if HAVE_EXP_UMR

static ucs_status_t
uct_ib_mlx5_exp_umr_fill_repeated(uct_ib_umr_t *umr, const uct_iov_t *iov,
                                  size_t iov_count)
{
    struct ibv_exp_send_wr *wr = &umr->wr;
    struct ibv_exp_mem_repeat_block *mem_rep_list;
    uint64_t base_addr = SIZE_MAX;
    int i;

    memset(wr, 0, sizeof(*wr));
    mem_rep_list = ucs_calloc(iov_count, sizeof(*mem_rep_list), "UMR memlist");
    if (mem_rep_list == NULL) {
        ucs_error("can't allocated mem_repeat_block for UMR");
        goto err_no_mem;
    }

    for(i = 0; i < iov_count; ++i) {
        mem_rep_list[i].byte_count = ucs_calloc(1,
                                                sizeof(mem_rep_list[i].byte_count[0]),
                                                "byte_count");
        if (mem_rep_list[i].byte_count == NULL) {
            ucs_error("failed to allocate byte_cnt for mem_rep_list");
            goto err_clean_mem_rep_list;
        }
        mem_rep_list[i].stride = ucs_calloc(1, sizeof(mem_rep_list[i].stride[0]),
                                            "stride");
        if (mem_rep_list[i].stride == NULL) {
            ucs_error("failed to allocate stride for mem_rep_list");
            goto err_clean_mem_rep_list;
        }
        mem_rep_list[i].mr            = ((uct_ib_mlx5_mem_t*)iov[i].memh)->mr;
        mem_rep_list[i].base_addr     = (uint64_t)iov[i].buffer;
        mem_rep_list[i].byte_count[0] = iov[i].length;
        mem_rep_list[i].stride[0]     = iov[i].stride;
        base_addr                     = ucs_min(mem_rep_list[i].base_addr, base_addr);
        ucs_info("fill_replist(%d): addr %p(%ld), len %ld, stride %ld mr %p (memh %p)",
                 i,iov[i].buffer,  mem_rep_list[i].base_addr , iov[i].length,
                 iov[i].stride, mem_rep_list[i].mr, iov[i].memh);
    }

    umr->base_addr                                   = base_addr;
    wr->ext_op.umr.umr_type                          = IBV_EXP_UMR_REPEAT;
    wr->ext_op.umr.mem_list.rb.mem_repeat_block_list = mem_rep_list;
    wr->ext_op.umr.mem_list.rb.repeat_count          = &umr->repeat_count;
    wr->ext_op.umr.mem_list.rb.stride_dim            = 1;

    return UCS_OK;

err_clean_mem_rep_list:
    for (i = 0; i < iov_count; ++i) {
        ucs_free(mem_rep_list[i].byte_count);
        ucs_free(mem_rep_list[i].stride);
    }
    ucs_free(mem_rep_list);
err_no_mem:
    return UCS_ERR_NO_MEMORY;
}

/* TODO: Support KSM */
static ucs_status_t
uct_ib_mlx5_exp_umr_fill_region(uct_ib_umr_t *umr, const uct_iov_t *iov,
                                size_t iov_count)
{
    struct ibv_exp_mem_region *mem_reg;
    uint64_t base_addr = SIZE_MAX;
    int i;

    mem_reg = ucs_calloc(iov_count, sizeof(*mem_reg), "UMR memreg");
    if (!mem_reg) {
        ucs_fatal("can't allocate memory region list for UMR");;
    }

    for (i = 0; i < iov_count; i++) {
        mem_reg[i].base_addr = (uint64_t)iov[i].buffer; /* mr->addr? */
        mem_reg[i].length    = iov[i].length;
        mem_reg[i].mr        = ((uct_ib_mlx5_mem_t*)iov[i].memh)->mr;
        base_addr            = ucs_min(mem_reg[i].base_addr, base_addr);
        ucs_info("fill_region(%d): addr %p(%ld), len %ld mr %p (memh %p)",
                 i,iov[i].buffer,  mem_reg[i].base_addr , mem_reg[i].length, mem_reg[i].mr, iov[i].memh);
    }

    umr->base_addr                           = base_addr;
    umr->wr.ext_op.umr.umr_type              = IBV_EXP_UMR_MR_LIST;
    umr->wr.ext_op.umr.mem_list.mem_reg_list = mem_reg;

    return UCS_OK;
}

ucs_status_t
uct_ib_mlx5_exp_umr_alloc(uct_ib_mlx5_md_t *md, const uct_iov_t *iov,
                          size_t iov_count, size_t repeat_count,
                          uct_ib_mem_t **memh_p)
{
    uct_ib_umr_t *umr;
    ucs_status_t status;
    unsigned klms_needed, umr_depth;

    if (!IBV_EXP_HAVE_UMR(&md->super.dev.dev_attr)) {
        return UCS_ERR_UNSUPPORTED;
    }

    if (iov_count == 0) {
        return UCS_ERR_INVALID_PARAM;
    }

    ucs_assert_always(repeat_count > 0);

    status = uct_ib_md_calc_required_klms(md, iov, iov_count, &klms_needed,
                                          &umr_depth);
    if (status != UCS_OK) {
        ucs_error("Invalid UMR format");
        return status;
    }

    umr = ucs_calloc(1, sizeof(*umr), "umr");
    if (umr == NULL) {
        ucs_fatal("can't allocate UMR");
    }
    memset(&umr->wr, 0, sizeof(umr->wr));

    umr->repeat_count = repeat_count;
    umr->depth        = umr_depth;
    umr->iov_count    = iov_count;
    umr->comp.count   = 1; /* for async reg */
    umr->memh.umr     = umr;
    umr->contig_memh  = ucs_derived_of(iov->memh, uct_ib_mlx5_mem_t); /* assume all iovs use the same memh for now */

    if (repeat_count == 1) { /* MRs list */
        status = uct_ib_mlx5_exp_umr_fill_region(umr, iov, iov_count);
    } else { /* stride/interleave */
        status = uct_ib_mlx5_exp_umr_fill_repeated(umr, iov, iov_count);
    }

    *memh_p = &umr->memh.super;

    return status;
}

ucs_status_t
uct_ib_mlx5_exp_umr_register(uct_ib_mlx5_md_t *md, uct_ib_mem_t *memh,
                             struct ibv_qp *qp, struct ibv_cq *cq, int sync)
{
    uct_ib_mlx5_mem_t *ib_memh       = ucs_derived_of(memh, uct_ib_mlx5_mem_t);
    struct ibv_wc wc                 = {};
    struct ibv_exp_create_mr_in mrin = {};
    uct_ib_umr_t *umr                = ib_memh->umr;
    struct ibv_exp_send_wr *wr       = &umr->wr;
    struct ibv_exp_send_wr *bad_wr;
    int ret;
    //size_t length;

    ucs_assert_always(sync); /* TODO: support nonblocking*/

    /* Create indirect MR */
    mrin.pd                     = md->super.pd;
    mrin.attr.create_flags      = IBV_EXP_MR_INDIRECT_KLMS;
    mrin.attr.exp_access_flags  = UCT_IB_MEM_ACCESS_FLAGS;
    mrin.attr.max_klm_list_size = umr->iov_count;
    umr->memh.mr = ibv_exp_create_mr(&mrin);
    printf("UMR reg: mr=%p, lkey=%d\n", umr->memh.mr, (int)umr->memh.mr->lkey);
    if (!umr) {
        ucs_error("ibv_exp_create_mr() failed: %m");
        return UCS_ERR_NO_MEMORY;
    }
/*
    length = 0;
    for(i = 0; i < umr->iovcnt; i++) {
        if (!iov[i].stride) {
            length += iov[i].length;
        } else {
            length += iov[i].length * iov[i].count;
        }
    }
*/
    /* Fill indirect MR */
    umr->is_inline = (umr->iov_count <= md->super.config.max_inline_klm_list);

    /* If the list exceeds max inline size, allocate a container object */
    if (umr->is_inline) {
        wr->ext_op.umr.memory_objects = NULL;
        wr->exp_send_flags           |= IBV_EXP_SEND_INLINE;
    } else {
        struct ibv_exp_mkey_list_container_attr in = {
            .pd                = md->super.pd,
            .mkey_list_type    = IBV_EXP_MKEY_LIST_TYPE_INDIRECT_MR,
            .max_klm_list_size = umr->iov_count
        };

        wr->ext_op.umr.memory_objects = ibv_exp_alloc_mkey_list_memory(&in);
        if (wr->ext_op.umr.memory_objects == NULL) {
            ucs_fatal("ibv_exp_alloc_mkey_list_memory(list_size=%ld) failed: %m",
                      umr->iov_count);
        }
    }

    wr->exp_opcode             = IBV_EXP_WR_UMR_FILL;
    wr->exp_send_flags        |= IBV_EXP_SEND_SIGNALED;
    wr->ext_op.umr.exp_access  = UCT_IB_MEM_ACCESS_FLAGS;
    wr->ext_op.umr.modified_mr = umr->memh.mr;
    wr->ext_op.umr.num_mrs     = umr->iov_count;
    wr->ext_op.umr.base_addr   = umr->base_addr;


    /* Post UMR */
    ret = ibv_exp_post_send(qp, wr, &bad_wr);
    if (ret) {
        ucs_fatal("ibv_exp_post_send(UMR_FILL) failed: %m");
    }
    ucs_info("num_mrs %d, base_addr %p, inline %d, type %d",
             wr->ext_op.umr.num_mrs,
             (void*)(uintptr_t)wr->ext_op.umr.base_addr, umr->is_inline,
             wr->ext_op.umr.umr_type);

    /* Wait for send UMR completion */
    for (;;) {
        ret = ibv_poll_cq(cq, 1, &wc);
        if (ret == 1) {
            if (wc.status != IBV_WC_SUCCESS) {
                ucs_fatal("UMR_FILL completed with error: %s vendor_err %d",
                          ibv_wc_status_str(wc.status), wc.vendor_err);
            }
            break;
        }
        if (ret < 0) {
            ucs_fatal("ibv_exp_poll_cq(umr_cq) failed: %m");
        }
    }
    ucs_info("UMR posted!!!");

    /* TODO: check that not needed */
    /* umr->mr->addr       = iov[0].buffer;
       umr->mr->length     = length; */

    uct_ib_memh_init_from_mr(&umr->memh.super, umr->memh.mr); /* init keys */
    umr->memh.umr         = umr;
    umr->memh.super.flags = UCT_IB_MEM_FLAG_NC_MR;
    umr->memh.umr_depth   = umr->depth;
    //umr->memh.mr->addr = umr->base_addr;

    if (wr->ext_op.umr.memory_objects != NULL) {
        ucs_assert_always(!umr->is_inline);
        ibv_exp_dealloc_mkey_list_memory(wr->ext_op.umr.memory_objects);
    }

    if (umr->repeat_count > 1) {
        ucs_free(wr->ext_op.umr.mem_list.rb.mem_repeat_block_list);
    } else {
        ucs_assert(umr->repeat_count == 1);
        ucs_free(wr->ext_op.umr.mem_list.mem_reg_list);
    }

    return UCS_OK;
}

ucs_status_t
uct_ib_mlx5_exp_umr_deregister(uct_ib_mlx5_md_t *md, uct_ib_mem_t *memh,
                               struct ibv_qp *qp, struct ibv_cq *cq)
{
    uct_ib_mlx5_mem_t *ib_memh = ucs_derived_of(memh, uct_ib_mlx5_mem_t);
    uct_ib_umr_t *umr          = ib_memh->umr;
    struct ibv_exp_send_wr *wr = &umr->wr;
    struct ibv_wc wc           = {};
    ucs_status_t status        = UCS_OK;
    struct ibv_exp_send_wr *bad_wr;
    int ret;

    memset(wr, 0, sizeof(*wr));
    wr->exp_opcode             = IBV_EXP_WR_UMR_INVALIDATE;
    wr->exp_send_flags         = IBV_EXP_SEND_INLINE |
                                 IBV_EXP_SEND_SIGNALED;
    wr->ext_op.umr.modified_mr = umr->memh.mr;

    /* Post UMR */
    ret = ibv_exp_post_send(qp, wr, &bad_wr);
    if (ret) {
        ucs_fatal("ibv_exp_post_send(UMR_INV) failed: %m");
    }

    /* Wait for send UMR completion */
    for (;;) {
        ret = ibv_poll_cq(cq, 1, &wc);
        if (ret == 1) {
            if (wc.status != IBV_WC_SUCCESS) {
                ucs_fatal("UMR_INV completed with error: %s vendor_err %d",
                          ibv_wc_status_str(wc.status), wc.vendor_err);
            }
            break;
        }
        if (ret < 0) {
            ucs_fatal("ibv_exp_poll_cq(umr_cq) failed: %m");
        }
    }

    if (umr->contig_memh->super.flags & ~UCT_IB_MEM_FLAG_NEED_DEREG) {
        umr->contig_memh->super.flags &= ~(UCT_IB_MEM_FLAG_NC_BASE_MR |
                                           UCT_IB_MEM_FLAG_NEED_DEREG);
        status = uct_ib_mlx5_dereg_key(&md->super, &umr->contig_memh->super);
    }

    ucs_free(umr);

    return status;
}

static ucs_status_t uct_ib_mlx5_mem_reg_nc(uct_ib_md_t *ib_md, const uct_iov_t *iov,
                                          size_t iovcnt, size_t repeat_count,
                                          uct_mem_h *memh_p)
{
#if HAVE_EXP_UMR
    uct_ib_mlx5_md_t *md = ucs_derived_of(ib_md, uct_ib_mlx5_md_t);
    ucs_status_t status;
    uct_ib_mem_t *memh;

    if ((memh_p == NULL) || (repeat_count == 0) || (iovcnt == 0)) {
        ucs_error("Invalid UMR parameters: memh_p %p, repeat_count %ld, iovcnt %ld",
                  memh_p, repeat_count, iovcnt);
        return UCS_ERR_INVALID_PARAM;
    }

    ucs_info("reg NC on MD(%p): %p, iovs %ld, repeat %ld",
             md, iov[0].buffer, iovcnt, repeat_count);

    status = uct_ib_mlx5_exp_umr_alloc(md, iov, iovcnt, repeat_count, &memh);
    if (ucs_unlikely(status != UCS_OK)) {
        return status;
    }

    status = uct_ib_mlx5_exp_umr_register(md, memh, md->umr_qp, md->umr_cq, 1);
    if (ucs_unlikely(status != UCS_OK)) {
        return status;
    }

    *memh_p = memh;
    return UCS_OK;
#else
    return UCS_ERR_UNSUPPORTED;
#endif
}

static ucs_status_t uct_ib_mlx5_mem_dereg_nc(uct_ib_md_t *ib_md, uct_mem_h memh)
{
#if HAVE_EXP_UMR
    uct_ib_mlx5_md_t *md  = ucs_derived_of(ib_md, uct_ib_mlx5_md_t);
    uct_ib_mem_t *ib_memh = (uct_ib_mem_t*)memh;
    ucs_status_t status;

    status = uct_ib_mlx5_exp_umr_deregister(md, ib_memh, md->umr_qp, md->umr_cq);
    if (status != UCS_OK) {
        ucs_error("failed to deregister NC memory: %s",
                  ucs_status_string(status));
    }

    ucs_info("UMR deregistered, md %p memh %p", ib_md, memh);

    return status;
#else
    return UCS_ERR_UNSUPPORTED;
#endif

}

static ucs_status_t
uct_ib_mlx5_exp_reg_indirect_mr(uct_ib_mlx5_md_t *md,
                                void *addr, size_t length,
                                struct ibv_exp_mem_region *mem_reg,
                                int list_size, uint32_t create_flags,
                                uint32_t umr_type, struct ibv_mr **mr_p)
{
    struct ibv_exp_send_wr wr, *bad_wr;
    struct ibv_exp_create_mr_in mrin;
    ucs_status_t status;
    struct ibv_mr *umr;
    struct ibv_wc wc;
    int ret;

    if (md->umr_qp == NULL) {
        status = UCS_ERR_UNSUPPORTED;
        goto err;
    }

    /* Create and fill memory key */
    memset(&mrin, 0, sizeof(mrin));
    memset(&wr, 0, sizeof(wr));

    mrin.pd                             = md->super.pd;
    wr.exp_opcode                       = IBV_EXP_WR_UMR_FILL;
    wr.exp_send_flags                   = IBV_EXP_SEND_SIGNALED;
    wr.ext_op.umr.exp_access            = UCT_IB_MEM_ACCESS_FLAGS;

    mrin.attr.create_flags              = create_flags;
    wr.ext_op.umr.umr_type              = umr_type;

    mrin.attr.exp_access_flags          = UCT_IB_MEM_ACCESS_FLAGS;
    mrin.attr.max_klm_list_size         = list_size;

    umr = ibv_exp_create_mr(&mrin);
    if (!umr) {
        ucs_error("ibv_exp_create_mr() failed: %m");
        status = UCS_ERR_NO_MEMORY;
        goto err;
    }

    wr.ext_op.umr.mem_list.mem_reg_list = mem_reg;
    wr.ext_op.umr.base_addr             = (uint64_t)(uintptr_t)addr;
    wr.ext_op.umr.num_mrs               = list_size;
    wr.ext_op.umr.modified_mr           = umr;

    /* If the list exceeds max inline size, allocate a container object */
    if (list_size > md->super.config.max_inline_klm_list) {
        struct ibv_exp_mkey_list_container_attr in = {
            .pd                = md->super.pd,
            .mkey_list_type    = IBV_EXP_MKEY_LIST_TYPE_INDIRECT_MR,
            .max_klm_list_size = list_size
        };

        wr.ext_op.umr.memory_objects = ibv_exp_alloc_mkey_list_memory(&in);
        if (wr.ext_op.umr.memory_objects == NULL) {
            ucs_error("ibv_exp_alloc_mkey_list_memory(list_size=%d) failed: %m",
                      list_size);
            status = UCS_ERR_IO_ERROR;
            goto err_free_umr;
        }
    } else {
        wr.ext_op.umr.memory_objects = NULL;
        wr.exp_send_flags           |= IBV_EXP_SEND_INLINE;
    }

    ucs_trace_data("UMR_FILL qp 0x%x lkey 0x%x base 0x%lx [addr %lx len %zu lkey 0x%x] list_size %d",
                   md->umr_qp->qp_num, wr.ext_op.umr.modified_mr->lkey,
                   wr.ext_op.umr.base_addr, mem_reg[0].base_addr,
                   mem_reg[0].length, mem_reg[0].mr->lkey, list_size);

    /* Post UMR */
    ret = ibv_exp_post_send(md->umr_qp, &wr, &bad_wr);
    if (ret) {
        ucs_error("ibv_exp_post_send(UMR_FILL) failed: %m");
        status = UCS_ERR_IO_ERROR;
        goto err_free_klm_container;
    }

    /* Wait for send UMR completion */
    for (;;) {
        ret = ibv_poll_cq(md->umr_cq, 1, &wc);
        if (ret < 0) {
            ucs_error("ibv_exp_poll_cq(umr_cq) failed: %m");
            status = UCS_ERR_IO_ERROR;
            goto err_free_klm_container;
        }
        if (ret == 1) {
            if (wc.status != IBV_WC_SUCCESS) {
                ucs_error("UMR_FILL completed with error: %s vendor_err %d",
                          ibv_wc_status_str(wc.status), wc.vendor_err);
                status = UCS_ERR_IO_ERROR;
                goto err_free_klm_container;
            }
            break;
        }
    }

    if (wr.ext_op.umr.memory_objects != NULL) {
        ibv_exp_dealloc_mkey_list_memory(wr.ext_op.umr.memory_objects);
    }

    umr->addr   = addr;
    umr->length = length;
    ucs_debug("UMR registered memory %p..%p on %s lkey 0x%x rkey 0x%x",
              umr->addr, UCS_PTR_BYTE_OFFSET(umr->addr, length),
              uct_ib_device_name(&md->super.dev),
              umr->lkey, umr->rkey);

    *mr_p = umr;

    return UCS_OK;

err_free_klm_container:
    if (wr.ext_op.umr.memory_objects != NULL) {
        ibv_exp_dealloc_mkey_list_memory(wr.ext_op.umr.memory_objects);
    }
err_free_umr:
    UCS_PROFILE_CALL(ibv_dereg_mr, umr);
err:
    return status;
}
#endif

ucs_status_t uct_ib_mlx5_exp_reg_ksm(uct_ib_mlx5_md_t *md,
                                     uct_ib_mlx5_ksm_data_t *ksm_data,
                                     size_t length, off_t off,
                                     struct ibv_mr **mr_p)
{
#if HAVE_EXP_UMR_KSM
    struct ibv_exp_mem_region *mem_reg;
    ucs_status_t status;
    int i;

    mem_reg = ucs_calloc(ksm_data->mr_num, sizeof(mem_reg[0]), "mem_reg");
    if (!mem_reg) {
        return UCS_ERR_NO_MEMORY;
    }

    for (i = 0; i < ksm_data->mr_num; i++) {
        mem_reg[i].base_addr = (uint64_t) (uintptr_t) ksm_data->mrs[i]->addr;
        mem_reg[i].length    = ksm_data->mrs[i]->length;
        mem_reg[i].mr        = ksm_data->mrs[i];
    }

    status = uct_ib_mlx5_exp_reg_indirect_mr(md, ksm_data->mrs[0]->addr + off,
                                             length, mem_reg, ksm_data->mr_num,
                                             IBV_EXP_MR_FIXED_BUFFER_SIZE,
                                             IBV_EXP_UMR_MR_LIST_FIXED_SIZE,
                                             mr_p);

    ucs_free(mem_reg);
    return status;
#else
    return UCS_ERR_UNSUPPORTED;
#endif
}

static ucs_status_t uct_ib_mlx5_exp_reg_atomic_key(uct_ib_md_t *ibmd,
                                                   uct_ib_mem_t *ib_memh)
{
#if HAVE_EXP_UMR
    uct_ib_mlx5_mem_t *memh = ucs_derived_of(ib_memh, uct_ib_mlx5_mem_t);
    uct_ib_mlx5_md_t *md = ucs_derived_of(ibmd, uct_ib_mlx5_md_t);
    off_t offset = uct_ib_md_atomic_offset(uct_ib_mlx5_md_get_atomic_mr_id(md));
    struct ibv_exp_mem_region *mem_reg = NULL;
    struct ibv_mr *mr = memh->mr;
    uint32_t create_flags, umr_type;
    ucs_status_t status;
    struct ibv_mr *umr;
    int i, list_size;
    size_t reg_length;

    if (memh->super.flags & UCT_IB_MEM_MULTITHREADED) {
        status = uct_ib_mlx5_exp_reg_ksm(md, memh->ksm_data, memh->mr->length,
                                         offset, &memh->ksm_data->atomic_mr);
        if (status == UCS_OK) {
            memh->super.atomic_rkey = memh->ksm_data->atomic_mr->rkey;
        }

        return status;
    }

    reg_length = UCT_IB_MD_MAX_MR_SIZE;
#if HAVE_EXP_UMR_KSM
    if ((md->super.dev.dev_attr.comp_mask & IBV_EXP_DEVICE_ATTR_COMP_MASK_2) &&
        (md->super.dev.dev_attr.comp_mask_2 & IBV_EXP_DEVICE_ATTR_UMR_FIXED_SIZE_CAPS) &&
        (md->super.dev.dev_attr.exp_device_cap_flags & IBV_EXP_DEVICE_UMR_FIXED_SIZE))
    {
        reg_length   = md->super.dev.dev_attr.umr_fixed_size_caps.max_entity_size;
        list_size    = ucs_div_round_up(mr->length, reg_length);
    } else if (mr->length < reg_length) {
        list_size                       = 1;
    } else {
        status       = UCS_ERR_UNSUPPORTED;
        goto err;
    }

    if (list_size > 1) {
        create_flags = IBV_EXP_MR_FIXED_BUFFER_SIZE;
        umr_type     = IBV_EXP_UMR_MR_LIST_FIXED_SIZE;
    } else {
        create_flags = IBV_EXP_MR_INDIRECT_KLMS;
        umr_type     = IBV_EXP_UMR_MR_LIST;
    }
#else
    if (mr->length >= reg_length) {
        status       = UCS_ERR_UNSUPPORTED;
        goto err;
    }

    list_size        = 1;
    create_flags     = IBV_EXP_MR_INDIRECT_KLMS;
    umr_type         = IBV_EXP_UMR_MR_LIST;
#endif

    mem_reg          = ucs_calloc(list_size, sizeof(mem_reg[0]), "mem_reg");
    if (!mem_reg) {
        status = UCS_ERR_NO_MEMORY;
        goto err;
    }

    for (i = 0; i < list_size; i++) {
        mem_reg[i].base_addr = (uintptr_t) mr->addr + i * reg_length;
        mem_reg[i].length    = reg_length;
        mem_reg[i].mr        = mr;
    }

    ucs_assert(list_size >= 1);
    mem_reg[list_size - 1].length = mr->length % reg_length;

    status = uct_ib_mlx5_exp_reg_indirect_mr(md, mr->addr + offset, mr->length,
                                             mem_reg, list_size, create_flags,
                                             umr_type, &umr);
    if (status != UCS_OK) {
        goto err;
    }

    memh->atomic_mr               = umr;
    memh->super.atomic_rkey       = umr->rkey;

err:
    ucs_free(mem_reg);
    return status;
#else
    return UCS_ERR_UNSUPPORTED;
#endif
}

static ucs_status_t uct_ib_mlx5_exp_dereg_atomic_key(uct_ib_md_t *ibmd,
                                                     uct_ib_mem_t *ib_memh)
{
#if HAVE_EXP_UMR
    uct_ib_mlx5_mem_t *memh = ucs_derived_of(ib_memh, uct_ib_mlx5_mem_t);
    int ret;

    ret = UCS_PROFILE_CALL(ibv_dereg_mr, memh->atomic_mr);
    if (ret != 0) {
        ucs_error("ibv_dereg_mr() failed: %m");
        return UCS_ERR_IO_ERROR;
    }

    return UCS_OK;
#else
    return UCS_ERR_UNSUPPORTED;
#endif
}

static ucs_status_t uct_ib_mlx5_exp_reg_multithreaded(uct_ib_md_t *ibmd,
                                                      void *address, size_t length,
                                                      uint64_t access,
                                                      uct_ib_mem_t *ib_memh)
{
#if HAVE_EXP_UMR_KSM
    uct_ib_mlx5_mem_t *memh = ucs_derived_of(ib_memh, uct_ib_mlx5_mem_t);
    uct_ib_mlx5_md_t *md = ucs_derived_of(ibmd, uct_ib_mlx5_md_t);
    size_t chunk = md->super.config.mt_reg_chunk;
    uct_ib_mlx5_ksm_data_t *ksm_data;
    size_t ksm_data_size;
    ucs_status_t status;
    struct ibv_mr *umr;
    int mr_num;

    if (!(md->super.dev.dev_attr.comp_mask & IBV_EXP_DEVICE_ATTR_COMP_MASK_2) ||
        !(md->super.dev.dev_attr.comp_mask_2 & IBV_EXP_DEVICE_ATTR_UMR_FIXED_SIZE_CAPS) ||
        !(md->super.dev.dev_attr.exp_device_cap_flags & IBV_EXP_DEVICE_UMR_FIXED_SIZE)) {
        return UCS_ERR_UNSUPPORTED;
    }

    mr_num        = ucs_div_round_up(length, chunk);
    ksm_data_size = (mr_num * sizeof(*ksm_data->mrs)) + sizeof(*ksm_data);
    ksm_data      = ucs_calloc(1, ksm_data_size, "ksm_data");
    if (!ksm_data) {
        status = UCS_ERR_NO_MEMORY;
        goto err;
    }

    ucs_trace("multithreaded register memory %p..%p chunks %d",
              address, UCS_PTR_BYTE_OFFSET(address, length), mr_num);

    ksm_data->mr_num = mr_num;
    status = uct_ib_md_handle_mr_list_multithreaded(ibmd, address, length,
                                                    access, chunk, ksm_data->mrs);
    if (status != UCS_OK) {
        goto err;
    }

    status = uct_ib_mlx5_exp_reg_ksm(md, ksm_data, length, 0, &umr);
    if (status != UCS_OK) {
        goto err_dereg;
    }

    memh->mr         = umr;
    memh->ksm_data   = ksm_data;
    memh->super.lkey = umr->lkey;
    memh->super.rkey = umr->rkey;
    return UCS_OK;

err_dereg:
    uct_ib_md_handle_mr_list_multithreaded(ibmd, address, length, UCT_IB_MEM_DEREG,
                                           chunk, ksm_data->mrs);
err:
    ucs_free(ksm_data);
    return status;
#else
    return UCS_ERR_UNSUPPORTED;
#endif
}

static ucs_status_t uct_ib_mlx5_exp_dereg_multithreaded(uct_ib_md_t *ibmd,
                                                        uct_ib_mem_t *ib_memh)
{
#if HAVE_EXP_UMR_KSM
    uct_ib_mlx5_mem_t *memh = ucs_derived_of(ib_memh, uct_ib_mlx5_mem_t);
    size_t chunk = ibmd->config.mt_reg_chunk;
    ucs_status_t s, status = UCS_OK;

    if (memh->super.flags & UCT_IB_MEM_FLAG_ATOMIC_MR) {
        s = uct_ib_dereg_mr(memh->ksm_data->atomic_mr);
        if (s != UCS_OK) {
            status = s;
        }
    }

    s = uct_ib_md_handle_mr_list_multithreaded(ibmd, memh->mr->addr,
                                               memh->mr->length,
                                               UCT_IB_MEM_DEREG, chunk,
                                               memh->ksm_data->mrs);
    if (s == UCS_ERR_UNSUPPORTED) {
        s = uct_ib_dereg_mrs(memh->ksm_data->mrs, memh->ksm_data->mr_num);
        if (s != UCS_OK) {
            status = s;
        }
    } else if (s != UCS_OK) {
        status = s;
    }

    s = uct_ib_dereg_mr(memh->mr);
    if (s != UCS_OK) {
        status = s;
    }

    ucs_free(memh->ksm_data);

    return status;
#else
    return UCS_ERR_UNSUPPORTED;
#endif
}

static uct_ib_md_ops_t uct_ib_mlx5_md_ops;

static ucs_status_t uct_ib_mlx5_exp_md_open(struct ibv_device *ibv_device,
                                            const uct_ib_md_config_t *md_config,
                                            uct_ib_md_t **p_md)
{
    ucs_status_t status = UCS_OK;
    struct ibv_context *ctx;
    uct_ib_device_t *dev;
    uct_ib_mlx5_md_t *md;

    ctx = ibv_open_device(ibv_device);
    if (ctx == NULL) {
        ucs_debug("ibv_open_device(%s) failed: %m", ibv_get_device_name(ibv_device));
        status = UCS_ERR_UNSUPPORTED;
        goto err;
    }

    md = ucs_calloc(1, sizeof(*md), "ib_mlx5_md");
    if (md == NULL) {
        status = UCS_ERR_NO_MEMORY;
        goto err_free_context;
    }

    dev              = &md->super.dev;
    dev->ibv_context = ctx;
    md->super.config = md_config->ext;

    status = uct_ib_device_query(dev, ibv_device);
    if (status != UCS_OK) {
        goto err_free;
    }

    if (!(uct_ib_device_spec(dev)->flags & UCT_IB_DEVICE_FLAG_MLX5_PRM)) {
        status = UCS_ERR_UNSUPPORTED;
        goto err_free;
    }

#if HAVE_DECL_IBV_EXP_DEVICE_DC_TRANSPORT && HAVE_STRUCT_IBV_EXP_DEVICE_ATTR_EXP_DEVICE_CAP_FLAGS
    if (dev->dev_attr.exp_device_cap_flags & IBV_EXP_DEVICE_DC_TRANSPORT) {
        dev->flags |= UCT_IB_DEVICE_FLAG_DC;
    }
#endif

#if IBV_HW_TM
    if (dev->dev_attr.tm_caps.capability_flags & IBV_EXP_TM_CAP_DC) {
        md->flags |= UCT_IB_MLX5_MD_FLAG_DC_TM;
    }
#endif

    if (UCT_IB_HAVE_ODP_IMPLICIT(&dev->dev_attr)) {
        dev->flags |= UCT_IB_DEVICE_FLAG_ODP_IMPLICIT;
    }

    if (IBV_EXP_HAVE_ATOMIC_HCA(&dev->dev_attr) ||
        IBV_EXP_HAVE_ATOMIC_GLOB(&dev->dev_attr) ||
        IBV_EXP_HAVE_ATOMIC_HCA_REPLY_BE(&dev->dev_attr))
    {
#ifdef HAVE_IB_EXT_ATOMICS
        if (dev->dev_attr.comp_mask & IBV_EXP_DEVICE_ATTR_EXT_ATOMIC_ARGS) {
            dev->ext_atomic_arg_sizes = dev->dev_attr.ext_atom.log_atomic_arg_sizes;
        }
#  if HAVE_MASKED_ATOMICS_ENDIANNESS
        if (dev->dev_attr.comp_mask & IBV_EXP_DEVICE_ATTR_MASKED_ATOMICS) {
            dev->ext_atomic_arg_sizes |=
                dev->dev_attr.masked_atomic.masked_log_atomic_arg_sizes;
            dev->ext_atomic_arg_sizes_be =
                dev->dev_attr.masked_atomic.masked_log_atomic_arg_sizes_network_endianness;
        }
#  endif
        dev->ext_atomic_arg_sizes &= UCS_MASK(dev->dev_attr.ext_atom.log_max_atomic_inline + 1);
#endif
        dev->atomic_arg_sizes = sizeof(uint64_t);
        if (IBV_EXP_HAVE_ATOMIC_HCA_REPLY_BE(&dev->dev_attr)) {
            dev->atomic_arg_sizes_be = sizeof(uint64_t);
        }
    }

#if HAVE_DECL_IBV_EXP_DEVICE_ATTR_PCI_ATOMIC_CAPS
    dev->pci_fadd_arg_sizes  = dev->dev_attr.pci_atomic_caps.fetch_add << 2;
    dev->pci_cswap_arg_sizes = dev->dev_attr.pci_atomic_caps.compare_swap << 2;
#endif

    md->super.ops = &uct_ib_mlx5_md_ops;
    status = uct_ib_md_open_common(&md->super, ibv_device, md_config);
    if (status != UCS_OK) {
        goto err_free;
    }

    status = uct_ib_mlx5_exp_md_umr_qp_create(md);
    if (status != UCS_OK && status != UCS_ERR_UNSUPPORTED) {
        goto err_free;
    }

    dev->flags |= UCT_IB_DEVICE_FLAG_MLX5_PRM;
    *p_md = &md->super;
    return UCS_OK;

err_free:
    ucs_free(md);
err_free_context:
    ibv_close_device(ctx);
err:
    return status;
}

void uct_ib_mlx5_exp_md_cleanup(uct_ib_md_t *ibmd)
{
#if HAVE_EXP_UMR
    uct_ib_mlx5_md_t *md = ucs_derived_of(ibmd, uct_ib_mlx5_md_t);

    if (md->umr_qp != NULL) {
        uct_ib_destroy_qp(md->umr_qp);
    }
    if (md->umr_cq != NULL) {
        ibv_destroy_cq(md->umr_cq);
    }
#endif
}

static uct_ib_md_ops_t uct_ib_mlx5_md_ops = {
    .open                = uct_ib_mlx5_exp_md_open,
    .cleanup             = uct_ib_mlx5_exp_md_cleanup,
    .memh_struct_size    = sizeof(uct_ib_mlx5_mem_t),
    .reg_key             = uct_ib_mlx5_reg_key,
    .dereg_key           = uct_ib_mlx5_dereg_key,
    .reg_atomic_key      = uct_ib_mlx5_exp_reg_atomic_key,
    .dereg_atomic_key    = uct_ib_mlx5_exp_dereg_atomic_key,
    .reg_multithreaded   = uct_ib_mlx5_exp_reg_multithreaded,
    .dereg_multithreaded = uct_ib_mlx5_exp_dereg_multithreaded,
    .mem_prefetch        = uct_ib_mlx5_mem_prefetch,
    .reg_nc              = uct_ib_mlx5_mem_reg_nc,
    .dereg_nc            = uct_ib_mlx5_mem_dereg_nc
};

UCT_IB_MD_OPS(uct_ib_mlx5_md_ops, 1);

