#ifndef RC_CAPTURE_H
#define RC_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/mcpwm_prelude.h"

// Maximum number of capture channels per MCPWM group (hardware limitation)
#define MAX_INPUTS_PER_GROUP 3

typedef struct {
    mcpwm_cap_channel_handle_t channel;  // MCPWM capture channel handle
    int gpio_num;                        // GPIO used for this input

    volatile uint32_t last_rise_us;      // Timestamp of last rising edge
    volatile uint32_t pulse_width_us;    // Measured pulse width
    volatile bool got_rise;              // True after rising edge, before falling edge
} rc_input_t;


typedef struct {
    int group_id;                                // MCPWM group ID
    mcpwm_cap_timer_handle_t timer;              // Capture timer for this group
    rc_input_t inputs[MAX_INPUTS_PER_GROUP];     // RC input channels
} rc_capture_group_t;

/**
 * Create new capture group for RC input.
 * 
 * Returns the initialized rc_capture_group_t
*/
rc_capture_group_t create_rc_capture_group(int group_id);

/**
 * Create a capture timer for one MCPWM group.
 */
void rc_capture_init_timer(rc_capture_group_t *cap, uint32_t resolution_hz);

/**
 * Add a channel to an RC capture group, to capture from a specific gpio_num.
*/
void rc_capture_add_channel(rc_capture_group_t *cap, int idx, int gpio_num);

/**
 * Enable capture channels and start the capture timer.
*/
void rc_capture_start(rc_capture_group_t *cap);

#endif // RC_CAPTURE_H
