//
// Created by marco on 2021-01-15.
//

/*
 * ppstest.c -- simple tool to monitor PPS timestamps
 *
 * Copyright (C) 2005-2007   Rodolfo Giometti <giometti@linux.it>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "pps_core.h"
#include "../gui.h"

static struct timespec offset_assert = {0, 0};

int find_source(char *path, pps_handle_t *handle, int *avail_mode)
{
    pps_params_t params;
    int ret;

    //PRINT_MSG_INFO("trying PPS source \"%s\"\n", path);

    /* Try to find the source by using the supplied "path" name */
    ret = open(path, O_RDWR);
    if (ret < 0) {
        return ret;
    }

    /* Open the PPS source (and check the file descriptor) */
    ret = time_pps_create(ret, handle);
    if (ret < 0) {

        return -1;
    }
     gui_status_wprintw(DEFAULT, "found PPS source \"%s\"\n", path);

    /* Find out what features are supported */
    ret = time_pps_getcap(*handle, avail_mode);
    if (ret < 0) {
        return -1;
    }
    if ((*avail_mode & PPS_CAPTUREASSERT) == 0) {
        return -1;
    }

    /* Capture assert timestamps */
    ret = time_pps_getparams(*handle, &params);
    if (ret < 0) {
        return -1;
    }
    params.mode |= PPS_CAPTUREASSERT;
    /* Override any previous offset if possible */
    if ((*avail_mode & PPS_OFFSETASSERT) != 0) {
        params.mode |= PPS_OFFSETASSERT;
        params.assert_offset = offset_assert;
    }
    ret = time_pps_setparams(*handle, &params);
    if (ret < 0) {
        return -1;
    }

    return 0;
}

int fetch_source(int i, pps_handle_t *handle, int *avail_mode, pps_info_t *infobuf)
{
    struct timespec timeout;
    int ret;

    /* create a zero-valued timeout */
    timeout.tv_sec = 3;
    timeout.tv_nsec = 0;

    retry:
    if (*avail_mode & PPS_CANWAIT) /* waits for the next event */
        ret = time_pps_fetch(*handle, PPS_TSFMT_TSPEC, infobuf,
                             &timeout);
    else {
        sleep(1);
        ret = time_pps_fetch(*handle, PPS_TSFMT_TSPEC, infobuf,
                             &timeout);
    }
    if (ret < 0) {
        if (ret == -EINTR) {
            goto retry;
        }

        return -1;
    }

    // gui_status_wprintw(DEFAULT, "source %d - "
    //        "assert %ld.%09ld, sequence: %ld - "
    //        "clear  %ld.%09ld, sequence: %ld\n",
    //        i,
    //        infobuf->assert_timestamp.tv_sec,
    //        infobuf->assert_timestamp.tv_nsec,
    //        infobuf->assert_sequence,
    //        infobuf->clear_timestamp.tv_sec,
    //        infobuf->clear_timestamp.tv_nsec, infobuf->clear_sequence);

    return 0;
}