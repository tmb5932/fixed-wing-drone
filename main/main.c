#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "pwm_output.h"
#include "rc_capture.h"

static const char *TAG = "MAIN";

#define SERVO_PULSE_GPIO             21        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define CAPTURE_RESOLUTION (1000000)

// GPIO assignments for RC receiver inputs
#define RC_THROTTLE_GPIO   10
#define RC_LEFT_GPIO       11
#define RC_RIGHT_GPIO      12
#define RC_MODE_GPIO       13


void app_main(void)
{
    // OUTPUT SETUP
    ESP_LOGI(TAG, "Creating output group 0.");
    group_t g0 = create_group(0);

    mcpwm_timer_handle_t timer0 = NULL;
    initialize_timer(&timer0, 0, SERVO_TIMEBASE_RESOLUTION_HZ, SERVO_TIMEBASE_PERIOD);

    ESP_LOGI(TAG, "Adding operator and generator/comparator.");
    int op_idx = add_operator(&g0, timer0);

    int gc_idx1 = add_gen_cmpr(&g0.operators[op_idx], SERVO_PULSE_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);    
    int gc_idx2 = add_gen_cmpr(&g0.operators[op_idx], 3, SERVO_TYPE, DEFAULT_STARTING_VALUE);    

    mcpwm_timer_start(&timer0);

    // INPUT SETUP

    ESP_LOGI(TAG, "Creating capture group 0.");
    rc_capture_group_t cap0 = create_rc_capture_group(0);
    rc_capture_init_timer(&cap0, CAPTURE_RESOLUTION);

    rc_capture_add_channel(&cap0, 0, RC_THROTTLE_GPIO);
    rc_capture_add_channel(&cap0, 1, RC_LEFT_GPIO);
    rc_capture_add_channel(&cap0, 2, RC_RIGHT_GPIO);

    rc_capture_start(&cap0);


    ESP_LOGI(TAG, "Creating capture group 1.");
    rc_capture_group_t cap1 = create_rc_capture_group(1);
    rc_capture_init_timer(&cap1, CAPTURE_RESOLUTION);

    rc_capture_add_channel(&cap1, 0, RC_MODE_GPIO);
    
    rc_capture_start(&cap1);

    int a1 = 0;

    while (1) {
        uint32_t p1 = servo_angle_to_compare(a1);
        uint32_t p2 = servo_angle_to_compare(-a1);

        update_comparator_value(g0.operators[op_idx].comparators[gc_idx1], p1);
        update_comparator_value(g0.operators[op_idx].comparators[gc_idx2], p2);

        a1 = (a1 == 25) ? -25 : 25;

        uint32_t throttle = cap0.inputs[0].pulse_width_us;
        uint32_t left     = cap0.inputs[1].pulse_width_us;
        uint32_t right    = cap0.inputs[2].pulse_width_us;
        uint32_t mode     = cap1.inputs[0].pulse_width_us;

        ESP_LOGI(TAG,
                 "thr=%lu us | left=%lu us | right=%lu us | mode=%lu us",
                 (unsigned long)throttle,
                 (unsigned long)left,
                 (unsigned long)right,
                 (unsigned long)mode);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
