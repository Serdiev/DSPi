/*
 * Crossover stubs for tools/peq_bench.
 *
 * dsp_pipeline.c calls into the crossover module for its default init and for
 * the sample-rate rebuild.  The PEQ benchmark does not exercise crossover
 * filters, so those entry points are stubbed out here and the three crossover
 * globals the module owns are provided as inert storage.
 */

#include "config.h"
#include "crossover.h"

XoverFilter   xover_filters[NUM_CHANNELS][MAX_XOVER_BANDS];
EqParamPacket xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS];
bool          channel_xover_bypassed[NUM_CHANNELS];

void xover_init_default_filters(void) {
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        channel_xover_bypassed[ch] = true;
        for (int i = 0; i < MAX_XOVER_BANDS; i++) {
            xover_filters[ch][i].num_sections = 0;
            xover_filters[ch][i].bypass = true;
            xover_recipes[ch][i].channel = (uint8_t)ch;
            xover_recipes[ch][i].band = (uint8_t)(XOVER_BAND_BASE + i);
            xover_recipes[ch][i].type = FILTER_FLAT;
            xover_recipes[ch][i].bypass = 0;
            xover_recipes[ch][i].freq = 1000.0f;
            xover_recipes[ch][i].Q = 0.707f;
            xover_recipes[ch][i].gain_db = 0.0f;
        }
    }
}

void xover_recalculate_all(float sample_rate) { (void)sample_rate; }

void xover_update_channel_bypass(uint8_t ch) { channel_xover_bypassed[ch] = true; }