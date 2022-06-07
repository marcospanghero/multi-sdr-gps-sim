#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
/* for PRIX64 */
#include <inttypes.h>
#include <lime/LimeSuite.h>
#include "gui.h"
#include "fifo.h"
#include "sdr.h"
#include "sdr_lime.h"

lms_device_t *device = NULL;
const lms_dev_info_t *info;
static atomic_bool lime_tx_thread_exit = false;

static const int gui_y_offset = 4;
static const int gui_x_offset = 2;
const char config_file[] = "gps-ini.ini";

static pthread_t lime_tx_thread;


int error()
{
    if (device != NULL)
    {
        LMS_Close(device);
        gui_status_wprintw(RED, "Error with LImeSDR.\n");
    }
    return -1;
}

int sdr_limesdr_init(simulator_t *simulator)
{
    gui_status_wprintw(GREEN, "INIT Lime\n");
    int n;
    double sample_rate_gps_hz;
    uint64_t freq_gps_hz;
    int y = gui_y_offset;

    if (simulator->sample_size == SC08)
    {
        gui_status_wprintw(YELLOW, "8 bit sample size requested. Reset to 16 bit with LIMESDR.\n");
    }
    simulator->sample_size = SC16;

    lms_info_str_t list[8];                // should be large enough to hold all detected devices
    if ((n = LMS_GetDeviceList(list)) < 0) // NULL can be passed to only get number of devices
        return error();
    gui_status_wprintw(GREEN, "Device found - Lime: %d\n", n);
    if (n < 1)
        return -1;

    // open the first device
    int ret = LMS_Open(&device, NULL, NULL);
    gui_status_wprintw(GREEN, "Open with status: %d\n", ret);
    LMS_LoadConfig(device, config_file);
    // Initialize device with default configuration
    // Do not use if you want to keep existing configuration
    // Use LMS_LoadConfig(device, "/path/to/file.ini") to load config from INI

    if (LMS_Init(device) != 0)
        error();
    LMS_EnableChannel(device, LMS_CH_TX, 1, false);
    LMS_EnableChannel(device, LMS_CH_RX, 0, true); /* LimeSuite bug workaround (needed since LimeSuite git rev 52d6129 - or v18.06.0) */
    LMS_EnableChannel(device, LMS_CH_RX, 1, false);
    // Enable TX channel,Channels are numbered starting at 0
    if (LMS_EnableChannel(device, LMS_CH_TX, 0, true) != 0)
        error();

    sample_rate_gps_hz = TX_SAMPLERATE;
    freq_gps_hz = TX_FREQUENCY;
    freq_gps_hz = freq_gps_hz * (10000000 - simulator->ppb) / 10000000;

    // Set sample rate
    if (LMS_SetSampleRate(device, sample_rate_gps_hz, 1) != 0)
        error();
    double actualHostSampleRate = 0.0;
    double actualRFSampleRate = 0.0;
    int getSampleRate = LMS_GetSampleRate(device, LMS_CH_TX, 0, &actualHostSampleRate, &actualRFSampleRate);
    gui_mvwprintw(TRACK, y++, gui_x_offset, "Sample rate host %lf - Real: %lf\n", actualHostSampleRate, actualRFSampleRate);
    // Set center frequency
    if (LMS_SetLOFrequency(device, LMS_CH_TX, 0, freq_gps_hz) != 0)
        error();

    // set TX gain

    if (simulator->tx_gain < TX_LIME_GAIN_MIN)
    {
        simulator->tx_gain = TX_LIME_GAIN_MIN;
    }
    else if (simulator->tx_gain > TX_LIME_GAIN_MAX)
    {
        simulator->tx_gain = TX_LIME_GAIN_MAX;
    }
    if (LMS_SetGaindB(device, LMS_CH_TX, 0, simulator->tx_gain) != 0)
        error();

    double current_freq;
    LMS_GetLOFrequency(device, LMS_CH_TX, 0, &current_freq);
    gui_mvwprintw(TRACK, y++, gui_x_offset,"Freq %f Hz", current_freq);

    //LMS_SetLPFBW(device, LMS_CH_TX, 0, TX_BW);

    if (!fifo_create(NUM_FIFO_BUFFERS, LIME_SDR_TRANSFER_BUFFER_SIZE, SC16))
    {
        gui_status_wprintw(RED, "Error creating IQ file fifo!\n");
        return -1;
    }

    const lms_dev_info_t *info = LMS_GetDeviceInfo(device);
    gui_mvwprintw(TRACK, y++, gui_x_offset, "Device Name: %s\n", info->deviceName);
    gui_mvwprintw(TRACK, y++, gui_x_offset, "Device Serial: %ld\n", info->boardSerialNumber);

    return 0;
}

static void *lime_tx_thread_ep(void *arg)
{
    (void)arg;
    thread_to_core(2);
    set_thread_name("limesdr-thread");

    int y = gui_y_offset + 4;

    

    int32_t ntx = 0;
    lms_stream_t tx_stream; // stream structure

    tx_stream.channel = 0;               // channel number
    tx_stream.fifoSize = 2*LIME_SDR_TRANSFER_BUFFER_SIZE;     // fifo size in samples
    tx_stream.throughputVsLatency = 1; // 0 min latency, 1 max throughput
    tx_stream.dataFmt = LMS_FMT_I16;     // integer16 samples
    tx_stream.isTx = true;               // TX channel

    LMS_SetupStream(device, &tx_stream);
    LMS_StartStream(&tx_stream);

    lms_stream_status_t stream_status;
    LMS_GetStreamStatus(&tx_stream, &stream_status);
    gui_mvwprintw(TRACK, y++, gui_x_offset,"Stream: %d\n", stream_status.active);

    short *tx_buffer = (short* )malloc(LIME_SDR_TRANSFER_BUFFER_SIZE);


    while (!lime_tx_thread_exit)
    {
        struct iq_buf *iq = fifo_dequeue();
        if (iq != NULL && iq->data16 != NULL)
        {
            LMS_GetStreamStatus(&tx_stream, &stream_status); //Obtain TX stream stats
            memcpy(tx_buffer, iq->data16, LIME_SDR_TRANSFER_BUFFER_SIZE);
            LMS_SendStream(&tx_stream, tx_buffer, LIME_SDR_TRANSFER_BUFFER_SIZE, NULL, 0);
            fifo_release(iq);
        }
        else
        {
            break;
        }
    }
    if (device != NULL)
    {
        LMS_StopStream(&tx_stream);
        LMS_DestroyStream(device, &tx_stream);
        LMS_EnableChannel(device, LMS_CH_TX, 0, false);

        // Close device
        if (LMS_Close(device) == 0)
            gui_status_wprintw(RED, "Device closed");
    }
    pthread_exit(NULL);
}

int sdr_limesdr_set_gain(const int gain)
{
    int g = gain;
    if (g > TX_LIME_GAIN_MAX)
        g = TX_LIME_GAIN_MIN;
    if (g < TX_LIME_GAIN_MIN)
        g = TX_LIME_GAIN_MAX;
    if (LMS_SetGaindB(device, LMS_CH_TX, 0, g) != 0)
        return error();
    return (uint)(g);
}

int sdr_limesdr_run(void)
{
    if (device == NULL)
    {
        return error();
    }
    fifo_wait_full();
    pthread_create(&lime_tx_thread, NULL, lime_tx_thread_ep, NULL);
    return 0;
}
void sdr_limesdr_close(void)
{
    lime_tx_thread_exit = true;
    fifo_halt();
    fifo_destroy();
    pthread_join(lime_tx_thread, NULL);
}
