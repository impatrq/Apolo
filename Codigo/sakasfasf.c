#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"

typedef struct {
    uint gpio;
    uint32_t min_pulse_us;
    uint32_t neutral_pulse_us;
    uint32_t max_pulse_us;
} servo_config_t;

#define SERVO_COUNT 5u
static const servo_config_t servos[SERVO_COUNT] = {
    {2, 500u, 1510u, 2500u},
    {4, 500u, 1510u, 2500u},
    {6, 500u, 1510u, 2500u},
    {8, 500u, 1510u, 2500u},
    {10, 500u, 1510u, 2500u}
};

#define SERVO_FREQUENCY_HZ 50u
#define SERVO_FRAME_US (1000000u / SERVO_FREQUENCY_HZ)
#define STEP_DELAY_MS 3000u

static const uint32_t finger_open_angle[SERVO_COUNT] = {
    170u, 170u, 170u, 170u, 110u
};
static const uint32_t finger_close_angle[SERVO_COUNT] = {
    0u, 0u, 0u, 0u, 0u
};

static uint32_t map_angle_to_pulse(uint32_t servo_index, uint32_t angle_deg) {
    if (angle_deg > 180u) angle_deg = 180u;
    const uint32_t min_pulse = servos[servo_index].min_pulse_us;
    const uint32_t max_pulse = servos[servo_index].max_pulse_us;
    return min_pulse + ((angle_deg * (max_pulse - min_pulse)) / 180u);
}

static void set_servo_angle(uint32_t servo_index, uint32_t angle_deg) {
    const uint32_t pulse_us = map_angle_to_pulse(servo_index, angle_deg);
    pwm_set_gpio_level(servos[servo_index].gpio, pulse_us);
}

static void open_hand(void) {
    for (uint32_t i = 0; i < SERVO_COUNT; ++i)
        set_servo_angle(i, finger_open_angle[i]);
}

static void close_hand(void) {
    for (uint32_t i = 0; i < SERVO_COUNT; ++i)
        set_servo_angle(i, finger_close_angle[i]);
}

static void initialize_pwm(void) {
    pwm_config config = pwm_get_default_config();
    float divider = (float)clock_get_hz(clk_sys) / 1000000.0f;
    if (divider < 1.0f) divider = 1.0f;
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, SERVO_FRAME_US - 1u);

    uint32_t initialized_slices = 0u;
    for (uint32_t i = 0; i < SERVO_COUNT; ++i) {
        gpio_set_function(servos[i].gpio, GPIO_FUNC_PWM);
        // Precargar el pulso de cierre antes de habilitar el PWM.
        const uint32_t close_pulse = map_angle_to_pulse(
            i, finger_close_angle[i]);
        pwm_set_gpio_level(servos[i].gpio, close_pulse);

        const uint slice = pwm_gpio_to_slice_num(servos[i].gpio);
        const uint32_t mask = 1u << slice;
        if ((initialized_slices & mask) == 0u) {
            pwm_init(slice, &config, true);
            initialized_slices |= mask;
        }
    }
}

static void close_and_wait(void) {
    close_hand();
    printf("Mano cerrada\n");
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
}

static void open_and_wait(void) {
    open_hand();
    printf("Mano abierta\n");
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
}

static void close_one_finger(uint32_t finger_index) {
    set_servo_angle(finger_index, finger_close_angle[finger_index]);
    printf("Dedo %lu cerrado\n", (unsigned long)(finger_index + 1u));
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
}

static void open_one_finger(uint32_t finger_index) {
    set_servo_angle(finger_index, finger_open_angle[finger_index]);
    printf("Dedo %lu abierto\n", (unsigned long)(finger_index + 1u));
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
}

static void demo_task(void *unused) {
    (void)unused;

    for (;;) {
        close_and_wait();
        open_and_wait();

        for (uint32_t finger = 0; finger < SERVO_COUNT; ++finger) {
            close_one_finger(finger);
        }

        close_and_wait();

        for (uint32_t finger = 0; finger < SERVO_COUNT; ++finger) {
            open_one_finger(finger);
        }

        open_and_wait();
    }
}

int main(void) {
    stdio_init_all();
    initialize_pwm();

    if (xTaskCreate(demo_task, "Demo", 512u, NULL, 2u, NULL) != pdPASS) {
        while (true) tight_loop_contents();
    }

    vTaskStartScheduler();
    while (true) tight_loop_contents();
}
