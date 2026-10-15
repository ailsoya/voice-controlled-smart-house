#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

using namespace ei;

#define SAMPLE_RATE_HZ          16000u
#define DECIMATION_RATE         64u
#define AUDIO_SYS_CLOCK_HZ      24576000u
#define PDM_DATA                P10_5
#define PDM_CLK                 P10_4

#define MIC_GAIN                20      // 0.5 dB steps, range -24..21
#define RECORD_SECONDS          2u
#define CONFIDENCE_THRESHOLD    0.60f

#define WINDOW_SAMPLES          (EI_CLASSIFIER_RAW_SAMPLE_COUNT)
#define RECORD_SAMPLES          (SAMPLE_RATE_HZ * RECORD_SECONDS)
#define FRAME_WORDS             1024u
#define WARMUP_FRAMES           4u

static cyhal_pdm_pcm_t pdm_pcm;
static cyhal_clock_t   audio_clock;
static cyhal_clock_t   pll_clock;

static volatile bool button_flag  = false;
static volatile bool pdm_pcm_flag = false;

static int16_t frame[FRAME_WORDS];
static int16_t recording[RECORD_SAMPLES];
static const int16_t *window_start = recording;

static cyhal_gpio_callback_data_t button_cb_data;

static void button_isr_handler(void *arg, cyhal_gpio_event_t event)
{
    (void)arg; (void)event;
    button_flag = true;
}

static void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void)arg; (void)event;
    pdm_pcm_flag = true;
}

static void clock_init(void)
{
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}

static void read_frame_blocking(void)
{
    pdm_pcm_flag = false;
    cyhal_pdm_pcm_read_async(&pdm_pcm, frame, FRAME_WORDS);
    while (!pdm_pcm_flag) { }
}

// Records stereo, keeps only the channel with signal
static void record_audio(void)
{
    cyhal_pdm_pcm_clear(&pdm_pcm);

    uint64_t energy_l = 0, energy_r = 0;
    for (uint32_t f = 0; f < WARMUP_FRAMES; f++) {
        read_frame_blocking();
        for (uint32_t i = 0; i + 1 < FRAME_WORDS; i += 2) {
            energy_l += (uint64_t)abs(frame[i]);
            energy_r += (uint64_t)abs(frame[i + 1]);
        }
    }
    const uint32_t ch = (energy_r > energy_l) ? 1u : 0u;

    uint32_t n = 0;
    while (n < RECORD_SAMPLES) {
        read_frame_blocking();
        for (uint32_t i = ch; i < FRAME_WORDS && n < RECORD_SAMPLES; i += 2) {
            recording[n++] = frame[i];
        }
    }
}

// Picks the loudest 1-second window (where the word is)
static void select_loudest_window(void)
{
    const uint32_t step = SAMPLE_RATE_HZ / 100u;
    uint64_t best = 0;
    uint32_t best_start = 0;

    for (uint32_t start = 0; start + WINDOW_SAMPLES <= RECORD_SAMPLES; start += step) {
        uint64_t e = 0;
        for (uint32_t i = start; i < start + WINDOW_SAMPLES; i += 4) {
            e += (uint64_t)abs(recording[i]);
        }
        if (e > best) { best = e; best_start = start; }
    }
    window_start = &recording[best_start];
}

static int get_audio_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&window_start[offset], out_ptr, length);
    return 0;
}

// printf in ModusToolbox has no %f
static void print_percent(float v)
{
    int p = (int)(v * 1000.0f + 0.5f);
    printf("%3d.%d%%", p / 10, p % 10);
}

int main(void)
{
    cy_rslt_t result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    __enable_irq();

    clock_init();

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    button_cb_data.callback     = button_isr_handler;
    button_cb_data.callback_arg = NULL;
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &button_cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);

    cyhal_pdm_pcm_cfg_t pdm_cfg;
    memset(&pdm_cfg, 0, sizeof(pdm_cfg));
    pdm_cfg.sample_rate     = SAMPLE_RATE_HZ;
    pdm_cfg.decimation_rate = DECIMATION_RATE;
    pdm_cfg.mode            = CYHAL_PDM_PCM_MODE_STEREO;
    pdm_cfg.word_length     = 16;
    pdm_cfg.left_gain       = MIC_GAIN;
    pdm_cfg.right_gain      = MIC_GAIN;

    result = cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_cfg);
    if (result != CY_RSLT_SUCCESS) {
        printf("ERROR: PDM init failed (0x%08lx)\r\n", (unsigned long)result);
        CY_ASSERT(0);
    }
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);

    printf("\r\n\r\nPress SW2, then say a color.\r\n");

    for (;;)
    {
        if (!button_flag) { continue; }
        button_flag = false;

        printf("\r\n>>> Listening... (2 s)\r\n");
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        record_audio();
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

        select_loudest_window();

        int peak = 0;
        for (uint32_t i = 0; i < WINDOW_SAMPLES; i++) {
            int a = abs(window_start[i]);
            if (a > peak) { peak = a; }
        }
        printf("Audio peak: %d (good: ~3000..25000)\r\n", peak);

        signal_t signal;
        signal.total_length = WINDOW_SAMPLES;
        signal.get_data     = &get_audio_data;

        ei_impulse_result_t ei_result;
        memset(&ei_result, 0, sizeof(ei_result));

        EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_result, false);
        if (err != EI_IMPULSE_OK) {
            printf("ERROR: run_classifier failed (%d)\r\n", (int)err);
            continue;
        }

        size_t best_i = 0;
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            printf("  %-8s ", ei_result.classification[i].label);
            print_percent(ei_result.classification[i].value);
            printf("\r\n");
            if (ei_result.classification[i].value > ei_result.classification[best_i].value) {
                best_i = i;
            }
        }
        printf("  (DSP %d ms, NN %d ms)\r\n", (int)ei_result.timing.dsp, (int)ei_result.timing.classification);

        const char *best_label = ei_result.classification[best_i].label;
        const float best_value = ei_result.classification[best_i].value;
        const bool is_color = (strcmp(best_label, "noise") != 0) && (strcmp(best_label, "unknown") != 0);

        if (best_value >= CONFIDENCE_THRESHOLD && is_color) {
            printf("==> Recognized: %s\r\n", best_label);
        } else if (best_value >= CONFIDENCE_THRESHOLD) {
            printf("==> (%s)\r\n", best_label);
        } else {
            printf("==> Not sure, try again\r\n");
        }
        printf("\r\nPress SW2, then say a color.\r\n");
    }
}
