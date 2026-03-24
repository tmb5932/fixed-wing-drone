#include "esp_log.h"
#include "esp_timer.h"
#include "rc_capture.h"

static const char *TAG = "RC_CAPTURE";
#define US_PER_SEC (1000000ULL)

/**
 * Interrupt that fires every time edge is found
*/
static bool rc_capture_cb(mcpwm_cap_channel_handle_t channel, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    rc_input_t* in = (rc_input_t*) user_data;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) { // rising edge
        in->last_rise_ticks = edata->cap_value;
        in->got_rise = true;
    } else if (edata->cap_edge == MCPWM_CAP_EDGE_NEG && in->got_rise) { // falling edge
        uint32_t width_ticks = edata->cap_value - in->last_rise_ticks;
        in->got_rise = false;

        // Convert ticks to microseconds using the actual capture timer resolution
        uint32_t width_us = (uint32_t)(((uint64_t)width_ticks * US_PER_SEC) / in->capture_resolution_hz);

        // Safety limiter
        if (width_us >= 800 && width_us <= 2200) {
            in->pulse_width_us = width_us;
        }
    }

    in->last_update_us = esp_timer_get_time();

    return false; // no task wake needed
}

rc_capture_group_t create_rc_capture_group(int group_id)
{
    if (group_id < 0 || group_id >= SOC_MCPWM_GROUPS) {
        ESP_LOGE(TAG, "Invalid capture group id: %d", group_id);
        abort();
    }

    rc_capture_group_t cap = {0};
    cap.group_id = group_id;
    return cap;
}

void rc_capture_init_timer(rc_capture_group_t *cap, uint32_t resolution_hz)
{
    if (cap->timer != NULL) {
        ESP_LOGE(TAG, "Capture timer already initialized");
        abort();
    }
    mcpwm_capture_timer_config_t timer_cfg = {
        .group_id = cap->group_id,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&timer_cfg, &cap->timer));
}

void rc_capture_add_channel(rc_capture_group_t *cap, int idx, int gpio_num)
{
    if (cap->timer == NULL) {
        ESP_LOGE(TAG, "Capture timer not initialized");
        abort();
    }
    
    if (idx < 0 || idx >= MAX_INPUTS_PER_GROUP) {
        ESP_LOGE(TAG, "Invalid channel index, max is %d", MAX_INPUTS_PER_GROUP);
        abort();
    }

    uint32_t actual_resolution_hz = 0;
    ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(cap->timer, &actual_resolution_hz));

    cap->inputs[idx].gpio_num = gpio_num;
    cap->inputs[idx].last_rise_ticks = 0;
    cap->inputs[idx].pulse_width_us = 0;
    cap->inputs[idx].capture_resolution_hz = actual_resolution_hz;
    cap->inputs[idx].got_rise = false;

    mcpwm_capture_channel_config_t chan_cfg = {
        .gpio_num = gpio_num,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = true,
    };

    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap->timer, &chan_cfg, &cap->inputs[idx].channel));

    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = rc_capture_cb,
    };

    ESP_ERROR_CHECK(
        mcpwm_capture_channel_register_event_callbacks(
            cap->inputs[idx].channel,
            &cbs,
            &cap->inputs[idx]
        )
    );
}

void rc_capture_start(rc_capture_group_t *cap)
{
    if (cap->timer == NULL) {
        ESP_LOGE(TAG, "Capture timer not initialized");
        abort();
    }

    for (int i = 0; i < MAX_INPUTS_PER_GROUP; i++) {
        if (cap->inputs[i].channel != NULL) {
            ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap->inputs[i].channel));
        } 
    }

    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap->timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap->timer));
}
