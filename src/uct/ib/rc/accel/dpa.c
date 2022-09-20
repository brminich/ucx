/**
* Copyright (c) NVIDIA CORPORATION & AFFILIATES, 2022. ALL RIGHTS RESERVED.
*
* See file LICENSE for terms.
*/

 #include <libflexio-dev/flexio_dev.h>

__FLEXIO_ENTRY_POINT_START
flexio_dev_poll_thread_t ucx_dpa_thread_entry_point;
void ucx_dpa_thread_entry_point(uint64_t thread_arg)
{

    flexio_dev_return();
}
__FLEXIO_ENTRY_POINT_END


