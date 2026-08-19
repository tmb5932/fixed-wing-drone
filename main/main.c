#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm_prelude.h"
#include "pwm_output.h"
#include "rc_capture.h"
#include "imu.h"
#include "gps.h"
#include "pid.h"

static const char *TAG = "MAIN";

#define CONTROL_TASK_HZ (100)
#define CONTROL_TASK_MS (1000 / CONTROL_TASK_HZ)

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define CAPTURE_RESOLUTION (80000000) // This is unchangeable on the esp32s3, so its always 80MHz

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
#define RC_RUDDER   (CH5_NUM - 1)
#define RC_SWITCH   (CH6_NUM - 1)
#define RC_DIAL     (CH4_NUM - 1)

// Airframe control-surface layout. Define AIRFRAME_AILEVON to build for a
// plane with combined aileron+elevator flaps instead of independent ones. 
// Leave undefined for a conventional airframe.
// #define AIRFRAME_AILEVON

static output_group_t out_groups[SOC_MCPWM_GROUPS];
static rc_capture_group_t cap_groups[SOC_MCPWM_GROUPS];

// i_limit=250 reserves at least half of the actuator's +/-500us range
// (SERVO_MIN/MAX_PULSEWIDTH_US relative to MIDDLE_SERVO_VAL) for P/D, so a
// wound-up I-term can't eat the whole output on its own. Placeholder like the
// gains themselves; re-tune once k_i is actually set to something nonzero.
pid_cfg_t ROLL_PID_CFG = {
    .k_p = 10,
    .k_i = 0,
    .k_d = 0.1,
    .i_limit = 250,
    .integral = 0,
    .last_err = 0,
    .first = true
};

pid_cfg_t PITCH_PID_CFG = {
    .k_p = 10,
    .k_i = 0,
    .k_d = 0.1,
    .i_limit = 250,
    .integral = 0,
    .last_err = 0,
    .first = true
};

// Conservative caps on commanded attitude. Retune once the airframe is
// flight-characterized / tested. These exist so nothing (i.e. a bug)
// can command a full 180 degree pitch through set_goal_roll_deg/set_goal_pitch_deg;
#define MAX_ROLL_GOAL_DEG  45.0f
#define MAX_PITCH_GOAL_DEG 20.0f

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float goal_roll_deg = 0.0f;
float goal_pitch_deg = 0.0f;

void set_goal_roll_deg(float deg) {
    goal_roll_deg = clampf(deg, -MAX_ROLL_GOAL_DEG, MAX_ROLL_GOAL_DEG);
}

void set_goal_pitch_deg(float deg) {
    goal_pitch_deg = clampf(deg, -MAX_PITCH_GOAL_DEG, MAX_PITCH_GOAL_DEG);
}

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

/**
 * Turns given channel number into its matching comparator handle
 * Returns handle of comparator for given channel
*/
mcpwm_cmpr_handle_t channel_to_comparator(int ch) {
    return out_groups[rc_channel_to_output_group(ch)].operators[rc_channel_to_output_op(ch)].comparators[rc_channel_to_output_cmpr(ch)];
}

int clip(int num, int min, int max) {
    if (num < min) {
        return min;
    } else if (num > max) {
        return max;
    } else {
        return num;
    }
}

void pass_through_inputs(uint32_t ch[NUM_RC_CHANNELS]) {
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        if (cap_groups[rc_channel_to_capture_group(i)].inputs[rc_channel_to_capture_channel(i)].last_update_us == 0) {
            continue;
        }

        update_comparator_value(channel_to_comparator(i), clip(ch[i], SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US));
    }
}

/**
 * Runs the autonomous PID outputs for one control cycle.
 * Returns false (and leaves the outputs untouched) if the IMU isn't ready or
 * its data couldn't be read this cycle, so the caller can fall back to manual.
 */
bool update_autonomous_outputs(void)
{
    if (!imu_ready) {
        return false;
    }

    const int MIDDLE_SERVO_VAL = 1500;
    const float dt_s = 1.0f / CONTROL_TASK_HZ;

    // Grab values from the IMU
    BaseType_t ret = xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(IMU_MUTEX_WAIT));

    // If we fail to take the mutex, log an error and return early. This is a critical failure, as we can't safely read the IMU data without the mutex.
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take imu_data_mutex! Error: %d", ret);
        return false;
    }

    float roll_deg = imu_data.roll;
    float pitch_deg = imu_data.pitch;
    xSemaphoreGive(imu_data_mutex);

    update_comparator_value(channel_to_comparator(RC_THROTTLE), 1000);

    float roll_cmd = pid_step(&ROLL_PID_CFG, roll_deg, goal_roll_deg, dt_s);
    float pitch_cmd = pid_step(&PITCH_PID_CFG, pitch_deg, goal_pitch_deg, dt_s);

#ifdef AIRFRAME_AILEVON
    // Combined surfaces: mix roll and pitch into left/right elevon outputs.
    // Sign convention (which physical output is "left" vs "right", and +/-
    // for roll) depends on servo mounting; verify direction on the bench,
    // same as the existing single-purpose aileron/elevator outputs already
    // require.
    int left = clip((int) (MIDDLE_SERVO_VAL + pitch_cmd - roll_cmd), SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US);
    int right = clip((int) (MIDDLE_SERVO_VAL + pitch_cmd + roll_cmd), SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US);
    update_comparator_value(channel_to_comparator(RC_AILERON), left);
    update_comparator_value(channel_to_comparator(RC_ELEVATOR), right);
#else
    int roll = clip((int) (MIDDLE_SERVO_VAL + roll_cmd), SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US);
    update_comparator_value(channel_to_comparator(RC_AILERON), roll);

    int tilt = clip((int) (MIDDLE_SERVO_VAL + pitch_cmd), SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US);
    update_comparator_value(channel_to_comparator(RC_ELEVATOR), tilt);
#endif

    update_comparator_value(channel_to_comparator(RC_RUDDER), MIDDLE_SERVO_VAL);
    // update_comparator_value(channel_to_comparator(5-1), 1500);
    // update_comparator_value(channel_to_comparator(4-1), 1500);

    return true;
}

// Once any critical autonomous-path failure happens (e.g. IMU not ready, or
// a failed mutex take), autonomous mode is locked out for the rest of this
// boot. There's no in-flight recovery for "the IMU never came up" or similar,
// so the plane needs a reset before autonomous can be trusted again.
static bool critical_fault_latched = false;

void control_task(void *arg) {
    uint32_t ch[NUM_RC_CHANNELS];
    while (1) {
        ch[RC_THROTTLE] = get_channel_pulse_width(RC_THROTTLE);
        ch[RC_AILERON] = get_channel_pulse_width(RC_AILERON);
        ch[RC_ELEVATOR] = get_channel_pulse_width(RC_ELEVATOR);
        ch[RC_RUDDER] = get_channel_pulse_width(RC_RUDDER);
        ch[RC_SWITCH] = get_channel_pulse_width(RC_SWITCH);
        ch[RC_DIAL] = get_channel_pulse_width(RC_DIAL);

        // printf("CH: %lu, %lu, %lu, %lu, %lu, %lu\n", ch[0], ch[1], ch[2], ch[3], ch[4], ch[5]);

        // No autonomous until we've heard from the radio at least once, otherwise we might start flying away on power up with bad imu data and no rc input
        bool radio_connected = cap_groups[rc_channel_to_capture_group(RC_SWITCH)].inputs[rc_channel_to_capture_channel(RC_SWITCH)].last_update_us != 0;

        // if not heard from radio in a while, go autonomous. Otherwise we fall from sky...
        bool stale_capture = is_stale(cap_groups[rc_channel_to_capture_group(RC_SWITCH)].inputs[rc_channel_to_capture_channel(RC_SWITCH)]);

        bool want_autonomous = !critical_fault_latched && radio_connected && (stale_capture || autonomous_mode_enabled(ch[RC_SWITCH]));

        if (want_autonomous && !update_autonomous_outputs()) {
            critical_fault_latched = true;
            want_autonomous = false;
        }

        if (!want_autonomous) {
            // manual pass-through mode from remote controller
            pass_through_inputs(ch);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_MS));
    }
}

void app_main(void)
{
    // Create the IMU and GPS tasks
    BaseType_t result = xTaskCreate(
        imu_task,
        "imu_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task! Error: %d", result);
        return;
    } else {
        ESP_LOGI(TAG, "IMU task created successfully");
    }

    result = xTaskCreate(
        gps_task,
        "gps_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS task! Error: %d", result);
        return;
    } else {
        ESP_LOGI(TAG, "GPS task created successfully");
    }

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
    
    add_operator(&out_groups[0], 2, timer0);
    add_gen_cmpr(&out_groups[0].operators[2], 0, CH5_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);    
    add_gen_cmpr(&out_groups[0].operators[2], 1, CH6_OUT_GPIO, SERVO_TYPE, DEFAULT_STARTING_VALUE);    
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
