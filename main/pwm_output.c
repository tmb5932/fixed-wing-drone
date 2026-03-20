#include "esp_log.h"
#include "pwm_output.h"

uint32_t servo_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

int starting_to_pulse_width(item_type_t type, comparator_starting_value_t val)
{
    int max = -1;
    int min = -1;
    int def = -1;

    if (type == SERVO_TYPE) {
        min = SERVO_MIN_PULSEWIDTH_US;
        max = SERVO_MAX_PULSEWIDTH_US;
        def = servo_angle_to_compare(0); // middle of range
    } else if (type == MOTOR_TYPE) {
        min = MOTOR_MIN_PULSEWIDTH_US;
        max = MOTOR_MAX_PULSEWIDTH_US;
        def = min; // motor should always default to off...
    } else {
        min = max = def = 1500; // most things are good roughly 1000-2000us, so 1500 is good safety if of unknown type;
    }

    switch (val) {
        case MIN_STARTING_VALUE:
            return min;
        case MAX_STARTING_VALUE:
            return max;
        default: 
            return def;
    }
}

/**
 * Creates and adds a new operator to the given group, running off the given timer.
 * 
 * Returns the index of the newly created operator.
*/
int add_operator(group_t* group_ptr, mcpwm_timer_handle_t timer)
{
    // Find first empty operator spot in group
    int op_idx = -1;
    for (int i = 0; i < MAX_OPERATORS; i++) {
        if (group_ptr->operators[i].op == NULL) {
            op_idx = i;
            break;
        }
    }
    if (op_idx == -1) {
        ESP_LOGE("MCPWM", "No free operator slot in group %lu", (unsigned long)group_ptr->id);
        abort();
    }

    mcpwm_operator_config_t operator_config = {
        .group_id = group_ptr->id, // operator must be in the same group to the timer
    };

    mcpwm_oper_handle_t* op_handle = &group_ptr->operators[op_idx].op;

    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, op_handle));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(*op_handle, timer));

    group_ptr->operators[op_idx].connected_timer = timer;

    return op_idx;
}

/**
 * Creates and adds new generator and comparator to the given operator. The generator will output over the given gpio_num.
 * 
 * Returns the index of the added comparator / generator (they always match index)
*/
int add_gen_cmpr(operator_t* operator_ptr, int gpio_num, item_type_t output_type, comparator_starting_value_t starting_pulse_width)
{
    int idx = -1;
    for (int i = 0; i < MAX_GENERATORS; i++) {
        if (operator_ptr->comparators[i] == NULL) {
            idx = i;
            break;
        }
    }

    if (idx == -1) {
        ESP_LOGE("MCPWM", "No free generator/comparator slot in operator");
        abort();
    }

    // Create new comparator
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(operator_ptr->op, &comparator_config, &operator_ptr->comparators[idx]));

    // Create new generator
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_num,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(operator_ptr->op, &generator_config, &operator_ptr->generators[idx]));

    // set the initial compare value; aka setting pulse width to the starting width
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(operator_ptr->comparators[idx], starting_to_pulse_width(output_type, starting_pulse_width)));

    // Set output signal to be high when counter < compare_val and low when counter > compare_val
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(operator_ptr->generators[idx],
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(operator_ptr->generators[idx],
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, operator_ptr->comparators[idx], MCPWM_GEN_ACTION_LOW)));

    return idx;
}

/**
 * Initializes and enables timer
*/
void initialize_timer(mcpwm_timer_handle_t* timer_ptr, int group_id, uint32_t resolution, uint32_t period)
{
    mcpwm_timer_config_t timer_config = {
        .group_id = group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = resolution,
        .period_ticks = period,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, timer_ptr));

    ESP_ERROR_CHECK(mcpwm_timer_enable(*timer_ptr));
}

/**
 * Starts timer for given timer handle. 
*/
void mcpwm_timer_start(mcpwm_timer_handle_t* timer_ptr)
{
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(*timer_ptr, MCPWM_TIMER_START_NO_STOP));
}

/**
 * Stops timer for given timer handle.
*/
void mcpwm_timer_stop(mcpwm_timer_handle_t* timer_ptr)
{
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(*timer_ptr, MCPWM_TIMER_STOP_FULL));
}

/**
 * Create a MCPWM group for the given ID.
*/
group_t create_group(int id)
{
    if (id < 0 || id >= SOC_MCPWM_GROUPS) {
        ESP_LOGE("MCPWM", "Invalid group id: %d", id);
        abort();
    }

    group_t group = {0};
    group.id = id;
    return group;
}

void update_comparator_value(mcpwm_cmpr_handle_t comparator, int new_val)
{
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, new_val));
}
