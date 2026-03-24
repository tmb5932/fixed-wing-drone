#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include "driver/mcpwm_prelude.h"
#include "pwm_output.h"
#include "rc_capture.h"

static const char *TAG = "MAIN";

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define CAPTURE_RESOLUTION (1000000) // This is unchangeable on the esp32s3, so its actually not this... its 80MHz

#define MCPWM_CAPTURE_CHANNELS_PER_GROUP (SOC_MCPWM_CAPTURE_TIMERS_PER_GROUP * SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER)
#define MCPWM_TRIGGERS_PER_GROUP (SOC_MCPWM_OPERATORS_PER_GROUP * SOC_MCPWM_TRIGGERS_PER_OPERATOR)
#define NUM_RC_CHANNELS (6)

// GPIO assignments for RC receiver inputs (i swear these are 6 almost neighboring pins)
#define CH1_IN_GPIO  21
#define CH2_IN_GPIO  47
#define CH3_IN_GPIO  48
#define CH4_IN_GPIO  35
#define CH5_IN_GPIO  36
#define CH6_IN_GPIO  37


// GPIO assignments for outputs to the peripherals
#define CH1_OUT_GPIO  9
#define CH2_OUT_GPIO  10
#define CH3_OUT_GPIO  11
#define CH4_OUT_GPIO  12
#define CH5_OUT_GPIO  13
#define CH6_OUT_GPIO  14

// translating from channel numbers to what it controls
#define CH1_NUM 1
#define CH2_NUM 2
#define CH3_NUM 3
#define CH4_NUM 4
#define CH5_NUM 5
#define CH6_NUM 6

// Used for indexing related values
#define RC_THROTTLE (CH3_NUM - 1)
#define RC_AILERON  (CH1_NUM - 1)
#define RC_ELEVATOR (CH2_NUM - 1)
#define RC_RUDDER   (CH4_NUM - 1)
#define RC_SWITCH   (CH5_NUM - 1)
#define RC_DIAL     (CH6_NUM - 1)

static output_group_t out_groups[SOC_MCPWM_GROUPS];
static rc_capture_group_t cap_groups[SOC_MCPWM_GROUPS];

bool autonomous_mode_enabled(uint32_t mode_us) {
    return (mode_us < 1500);
}

/* INPUT CONVERSIONS (2 groups with 3 capture channels each) */
/**
 * Convert rc channel number to capture channel number
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in) 
*/
int rc_channel_to_capture_channel(int ch)
{
    if (ch < 0 || ch >= NUM_RC_CHANNELS) { ESP_LOGE(TAG, "Invalid channel number: %d; should be in [0, %d)", ch, NUM_RC_CHANNELS); abort(); }
    return ch % MCPWM_CAPTURE_CHANNELS_PER_GROUP; 
}

/**
 * Convert rc channel number to capture group number
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in) 
*/
int rc_channel_to_capture_group(int ch)
{
    if (ch < 0 || ch >= NUM_RC_CHANNELS) { ESP_LOGE(TAG, "Invalid channel number: %d; should be in [0, %d)", ch, NUM_RC_CHANNELS); abort(); }
    return ch / MCPWM_CAPTURE_CHANNELS_PER_GROUP;
}

/* OUTPUT CONVERSIONS (2 groups with 3 operators each, with each operator having 2 triggers) */
/**
 * Convert rc channel number to output group number
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in) 
*/
int rc_channel_to_output_group(int ch)
{
    if (ch < 0 || ch >= NUM_RC_CHANNELS) { ESP_LOGE(TAG, "Invalid channel number: %d; should be in [0, %d)", ch, NUM_RC_CHANNELS); abort(); }
    return ch / MCPWM_TRIGGERS_PER_GROUP;
}

/**
 * Convert rc channel number to output operator number
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in) 
*/
int rc_channel_to_output_op(int ch)
{
    if (ch < 0 || ch >= NUM_RC_CHANNELS) { ESP_LOGE(TAG, "Invalid channel number: %d; should be in [0, %d)", ch, NUM_RC_CHANNELS); abort(); }
    return ch / SOC_MCPWM_GENERATORS_PER_OPERATOR;
}

/**
 * Convert rc channel number to output comparator / generator number
 * The ch parameter should be 0-indexed (subtract 1 from it before passing in) 
*/
int rc_channel_to_output_cmpr(int ch)
{
    if (ch < 0 || ch >= NUM_RC_CHANNELS) { ESP_LOGE(TAG, "Invalid channel number: %d; should be in [0, %d)", ch, NUM_RC_CHANNELS); abort(); }
    return ch % SOC_MCPWM_COMPARATORS_PER_OPERATOR;
}

uint32_t get_channel_pulse_width(int ch) {
    return cap_groups[rc_channel_to_capture_group(ch)].inputs[rc_channel_to_capture_channel(ch)].pulse_width_us;
}

/**
 * Check if the given input_capture is stale, aka haven't recieved any signal from reciever in 200 milliseconds.
 * Returns true if the input_capture is stale, else false
*/
bool is_stale(rc_input_t input_capture) {
    return (esp_timer_get_time() - input_capture.last_update_us) > 200000;
}

void pass_through_inputs(uint32_t ch[NUM_RC_CHANNELS]) {
    update_comparator_value(out_groups[rc_channel_to_output_group(RC_THROTTLE)].operators[rc_channel_to_output_op(RC_THROTTLE)].comparators[rc_channel_to_output_cmpr(RC_THROTTLE)], ch[RC_THROTTLE]);
    update_comparator_value(out_groups[rc_channel_to_output_group(RC_AILERON)].operators[rc_channel_to_output_op(RC_AILERON)].comparators[rc_channel_to_output_cmpr(RC_AILERON)], ch[RC_AILERON]);
    update_comparator_value(out_groups[rc_channel_to_output_group(RC_ELEVATOR)].operators[rc_channel_to_output_op(RC_ELEVATOR)].comparators[rc_channel_to_output_cmpr(RC_ELEVATOR)], ch[RC_ELEVATOR]);
    update_comparator_value(out_groups[rc_channel_to_output_group(RC_RUDDER)].operators[rc_channel_to_output_op(RC_RUDDER)].comparators[rc_channel_to_output_cmpr(RC_RUDDER)], ch[RC_RUDDER]);
    // Only using 4 outputs, last 2 aren't even enabled
    // update_comparator_value(out_groups[rc_channel_to_output_group(5)].operators[rc_channel_to_output_op(5-1)].comparators[rc_channel_to_output_cmpr(5-1)], ch[5-1]);
    // update_comparator_value(out_groups[rc_channel_to_output_group(6)].operators[rc_channel_to_output_op(6-1)].comparators[rc_channel_to_output_cmpr(6-1)], ch[6-1]);
}

float t = 0.0f;
float dt = 0.1f;   // controls speed

void step_autonomous_pid(void)
{
    float val = sinf(t);

    uint32_t pwm = (uint32_t)(1500 + 500 * val);

    t += dt;

    if (t > 2 * M_PI) {
        t -= 2 * M_PI;
    }

    update_comparator_value(out_groups[rc_channel_to_output_group(RC_THROTTLE)].operators[rc_channel_to_output_op(RC_THROTTLE)].comparators[rc_channel_to_output_cmpr(RC_THROTTLE)], 1000);
    update_comparator_value(out_groups[rc_channel_to_output_group(RC_AILERON)].operators[rc_channel_to_output_op(RC_AILERON)].comparators[rc_channel_to_output_cmpr(RC_AILERON)], pwm);
    update_comparator_value(out_groups[rc_channel_to_output_group(RC_ELEVATOR)].operators[rc_channel_to_output_op(RC_ELEVATOR)].comparators[rc_channel_to_output_cmpr(RC_ELEVATOR)], pwm);
    update_comparator_value(out_groups[rc_channel_to_output_group(RC_RUDDER)].operators[rc_channel_to_output_op(RC_RUDDER)].comparators[rc_channel_to_output_cmpr(RC_RUDDER)], pwm);
    // Only using 4 outputs, last 2 aren't even enabled
    // update_comparator_value(out_groups[rc_channel_to_output_group(5)].operators[rc_channel_to_output_op(5)].comparators[rc_channel_to_output_cmpr(5)], 1500);
    // update_comparator_value(out_groups[rc_channel_to_output_group(6)].operators[rc_channel_to_output_op(6)].comparators[rc_channel_to_output_cmpr(6)], 1500);
}

void control_task(void *arg) {
    while (1) {
        uint32_t ch[NUM_RC_CHANNELS];
        ch[RC_THROTTLE] = get_channel_pulse_width(RC_THROTTLE);
        ch[RC_AILERON] = get_channel_pulse_width(RC_AILERON);
        ch[RC_ELEVATOR] = get_channel_pulse_width(RC_ELEVATOR);
        ch[RC_RUDDER] = get_channel_pulse_width(RC_RUDDER);
        ch[RC_SWITCH] = get_channel_pulse_width(RC_SWITCH);
        // ch[RC_DIAL] = cap_groups[rc_channel_to_capture_group(RC_DIAL)].inputs[rc_channel_to_capture_channel(RC_DIAL)].pulse_width_us;

        // if not heard from radio in a while, go autonomous. Otherwise we fall from sky...
        bool stale_capture = is_stale(cap_groups[rc_channel_to_capture_group(RC_SWITCH)].inputs[rc_channel_to_capture_channel(RC_SWITCH)]);
        if (stale_capture || autonomous_mode_enabled(ch[RC_SWITCH])) {
            step_autonomous_pid();
        } else {
            // manual pass-through mode from remote controller
            pass_through_inputs(ch);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    // OUTPUT SETUP
    ESP_LOGI(TAG, "Creating output group 0.");
    out_groups[0] = create_group(0);

    mcpwm_timer_handle_t timer0 = NULL;
    initialize_timer(&timer0, 0, SERVO_TIMEBASE_RESOLUTION_HZ, SERVO_TIMEBASE_PERIOD);

    ESP_LOGI(TAG, "Adding operator and generator/comparator.");
    add_operator(&out_groups[0], 0, timer0);
    add_gen_cmpr(&out_groups[0].operators[0], 0, CH1_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);    
    add_gen_cmpr(&out_groups[0].operators[0], 1, CH2_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);    

    add_operator(&out_groups[0], 1, timer0);
    add_gen_cmpr(&out_groups[0].operators[1], 0, CH3_OUT_GPIO, MOTOR_TYPE, MIN_STARTING_VALUE);    
    add_gen_cmpr(&out_groups[0].operators[1], 1, CH4_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);    
    
    // add_operator(&out_groups[0], 2, timer0);
    // add_gen_cmpr(&out_groups[0].operators[2], 0, CH5_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);    
    // add_gen_cmpr(&out_groups[0].operators[2], 1, CH6_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);    
    mcpwm_timer_start(&timer0);

    // INPUT SETUP
    ESP_LOGI(TAG, "Creating capture group 0.");
    cap_groups[0] = create_rc_capture_group(0);
    rc_capture_init_timer(&cap_groups[0], CAPTURE_RESOLUTION); // Capture resolution not used on esp32s3

    rc_capture_add_channel(&cap_groups[0], 0, CH1_IN_GPIO);
    rc_capture_add_channel(&cap_groups[0], 1, CH2_IN_GPIO);
    rc_capture_add_channel(&cap_groups[0], 2, CH3_IN_GPIO);

    rc_capture_start(&cap_groups[0]);

    ESP_LOGI(TAG, "Creating capture group 1.");
    cap_groups[1] = create_rc_capture_group(1);
    rc_capture_init_timer(&cap_groups[1], CAPTURE_RESOLUTION); // Capture resolution not used on esp32s3

    rc_capture_add_channel(&cap_groups[1], 0, CH4_IN_GPIO);
    rc_capture_add_channel(&cap_groups[1], 1, CH5_IN_GPIO);
    rc_capture_add_channel(&cap_groups[1], 2, CH6_IN_GPIO);

    rc_capture_start(&cap_groups[1]);

    xTaskCreatePinnedToCore(control_task, "control", 8096, NULL, 1, NULL, 1);
    return;
}
