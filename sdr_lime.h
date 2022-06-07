/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF, ADLAM-Pluto or LimeSDR.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2021 Mictronics
 * Distributed under the MIT License.
 *
 */

#ifndef SDR_LIME_H
#define SDR_LIME_H

#include "gps-sim.h"

#define TX_VGA1   0
#define TX_LIME_GAIN_NORM_MIN 0
#define TX_LIME_GAIN_NORM_MAX 1
#define TX_LIME_GAIN_MIN 0
#define TX_LIME_GAIN_MAX 73

int sdr_limesdr_init(simulator_t *simulator);
void sdr_limesdr_close(void);
int sdr_limesdr_run(void);
int sdr_limesdr_set_gain( const int gain);
#endif /* SDR_LIME_H */