#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include "driver/mcpwm_prelude.h"

#define PULSEWIDTH_DEFAULT_MIN_US 1000
#define PULSEWIDTH_DEFAULT_MAX_US 2000

// The hard floor/ceiling setup-mode calibration can never be pushed past
#define PULSEWIDTH_ABS_MIN_US 750
#define PULSEWIDTH_ABS_MAX_US 2250

#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define MAX_OPERATORS SOC_MCPWM_OPERATORS_PER_GROUP
#define MAX_GENERATORS SOC_MCPWM_GENERATORS_PER_OPERATOR
#define MCPWM_TRIGGERS_PER_GROUP (SOC_MCPWM_OPERATORS_PER_GROUP * SOC_MCPWM_TRIGGERS_PER_OPERATOR)

typedef struct {
    mcpwm_oper_handle_t op;
    mcpwm_timer_handle_t connected_timer;
    mcpwm_gen_handle_t generators[2];
    mcpwm_cmpr_handle_t comparators[2];
} operator_t;

typedef struct {
    uint32_t id;
    operator_t operators[MAX_OPERATORS];
} output_group_t;

typedef enum {
    SERVO_TYPE,
    MOTOR_TYPE
} item_type_t;

typedef enum {
    MAX_STARTING_VALUE,
    MIN_STARTING_VALUE,
    DEFAULT_STARTING_VALUE
} comparator_starting_value_t;

uint32_t servo_angle_to_compare(int angle);

int starting_to_pulse_width(item_type_t type, comparator_starting_value_t val);

void add_operator(output_group_t* group_ptr, int op_idx, mcpwm_timer_handle_t timer);

void add_gen_cmpr(operator_t* operator_ptr, int idx, int gpio_num, item_type_t output_type, comparator_starting_value_t starting_pulse_width);

void initialize_timer(mcpwm_timer_handle_t* timer_ptr, int group_id, uint32_t resolution, uint32_t period);

void mcpwm_timer_start(mcpwm_timer_handle_t* timer_ptr);

void mcpwm_timer_stop(mcpwm_timer_handle_t* timer_ptr);

output_group_t create_group(int id);

void update_comparator_value(mcpwm_cmpr_handle_t comparator, int new_val);

#endif // PWM_OUTPUT_H
