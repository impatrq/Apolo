#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct {
    uint gpio;
    uint32_t min_pulse_us;
    uint32_t neutral_pulse_us;
    uint32_t max_pulse_us;
} servo_config_t;

typedef struct {
    uint16_t adc;
    uint16_t baseline;
    uint32_t envelope;
} emg_sample_t;

typedef struct {
    emg_sample_t sample;
    bool calibrated;
    bool hand_closed;
    uint32_t activation_count;
} app_status_t;

#define SERVO_COUNT 6u
static const servo_config_t servos[SERVO_COUNT] = {
    {0, 500u, 1510u, 2500u},
    {2, 500u, 1510u, 2500u},
    {4, 500u, 1510u, 2500u},
    {6, 500u, 1510u, 2500u},
    {8, 500u, 1510u, 2500u},
    {10, 500u, 1510u, 2500u}
};

#define SERVO_FREQUENCY_HZ       50u
#define SERVO_FRAME_US           (1000000u / SERVO_FREQUENCY_HZ)
#define EMG_ADC_GPIO             26u
#define EMG_ADC_INPUT            0u
#define EMG_SAMPLE_PERIOD_MS     5u
#define EMG_LOG_INTERVAL_MS      250u
#define EMG_BASELINE_SAMPLES     100u
#define EMG_REFRACTORY_MS        300u
#define EMG_THRESHOLD            50u
#define EMG_RELEASE_THRESHOLD    25u
#define EMG_ENVELOPE_DIVISOR     4u
#define EMG_BASELINE_DIVISOR     64u

#if EMG_RELEASE_THRESHOLD >= EMG_THRESHOLD
#error "EMG_RELEASE_THRESHOLD must be lower than EMG_THRESHOLD"
#endif

static const uint32_t finger_open_angle[5] = {170u, 170u, 170u, 170u, 110u};
static const uint32_t finger_close_angle[5] = {0u, 0u, 0u, 0u, 0u};

static QueueHandle_t sample_queue;
static app_status_t app_status;

static inline uint32_t servo_pulse_to_level(uint32_t pulse_us) {
    return pulse_us;
}

static uint32_t map_angle_to_pulse(uint32_t servo_index, uint32_t angle_deg) {
    if (angle_deg > 180u) angle_deg = 180u;
    const uint32_t min_pulse = servos[servo_index].min_pulse_us;
    const uint32_t max_pulse = servos[servo_index].max_pulse_us;
    return min_pulse + ((angle_deg * (max_pulse - min_pulse)) / 180u);
}

static void set_servo_angle(uint32_t servo_index, uint32_t angle_deg,
                            uint32_t *pulse_us) {
    pulse_us[servo_index] = map_angle_to_pulse(servo_index, angle_deg);
    pwm_set_gpio_level(servos[servo_index].gpio,
    servo_pulse_to_level(pulse_us[servo_index]));
}

static void open_hand(uint32_t *pulse_us) {
    for (uint32_t i = 0; i < 5u; ++i)
        set_servo_angle(i, finger_open_angle[i], pulse_us);
}

static void close_hand(uint32_t *pulse_us) {
    for (uint32_t i = 0; i < 5u; ++i)
        set_servo_angle(i, finger_close_angle[i], pulse_us);
}

static void initialize_pwm(uint32_t *pulse_us) {
    pwm_config config = pwm_get_default_config();
    float divider = (float)clock_get_hz(clk_sys) / 1000000.0f;
    if (divider < 1.0f) divider = 1.0f;
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, SERVO_FRAME_US - 1u);

    for (uint32_t i = 0; i < SERVO_COUNT; ++i) {
        gpio_set_function(servos[i].gpio, GPIO_FUNC_PWM);
        pulse_us[i] = servos[i].neutral_pulse_us;
    }

    // Configurar los niveles primero y luego inicializar cada slice una sola vez.
    for (uint32_t i = 0; i < SERVO_COUNT; ++i)
        pwm_set_gpio_level(servos[i].gpio,
        servo_pulse_to_level(pulse_us[i]));

    uint32_t initialized_slices = 0u;
    for (uint32_t i = 0; i < SERVO_COUNT; ++i) {
        const uint slice = pwm_gpio_to_slice_num(servos[i].gpio);
        const uint32_t mask = 1u << slice;
        if ((initialized_slices & mask) == 0u) {
            pwm_init(slice, &config, true);
            initialized_slices |= mask;
        }
    }
}

static void emg_task(void *unused) {
    (void)unused;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t sum = 0u;

    // Calibración de reposo: mantener el músculo relajado durante ~0,5 s.
    for (uint32_t i = 0; i < EMG_BASELINE_SAMPLES; ++i) {
        sum += adc_read();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(EMG_SAMPLE_PERIOD_MS));
    }

    uint16_t baseline = (uint16_t)(sum / EMG_BASELINE_SAMPLES);
    uint32_t envelope = 0u;
    emg_sample_t sample = {.adc = 0u, .baseline = baseline, .envelope = 0u};

    taskENTER_CRITICAL();
    app_status.sample = sample;
    app_status.calibrated = true;
    taskEXIT_CRITICAL();

    for (;;) {
        sample.adc = adc_read();
        const int32_t difference = (int32_t)sample.adc - (int32_t)baseline;
        const uint32_t rectified = (uint32_t)(difference < 0 ? -difference : difference);
        envelope = ((EMG_ENVELOPE_DIVISOR - 1u) * envelope + rectified)
        / EMG_ENVELOPE_DIVISOR;
        sample.envelope = envelope;

        if (envelope <= EMG_RELEASE_THRESHOLD) {
            int32_t adjust = ((int32_t)sample.adc - (int32_t)baseline)
            / (int32_t)EMG_BASELINE_DIVISOR;
            if (adjust == 0 && sample.adc != baseline)
                adjust = sample.adc > baseline ? 1 : -1;
            baseline = (uint16_t)((int32_t)baseline + adjust);
        }
        sample.baseline = baseline;

        taskENTER_CRITICAL();
        app_status.sample = sample;
        taskEXIT_CRITICAL();

        // Cola de longitud 1: el control procesa siempre la muestra más reciente.
        (void)xQueueOverwrite(sample_queue, &sample);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(EMG_SAMPLE_PERIOD_MS));
    }
}

static void control_task(void *unused) {
    (void)unused;
    uint32_t pulse_us[SERVO_COUNT];
    for (uint32_t i = 0; i < SERVO_COUNT; ++i)
        pulse_us[i] = servos[i].neutral_pulse_us;
    open_hand(pulse_us);

    bool hand_closed = false;
    bool armed = true;
    TickType_t last_activation = 0;
    const TickType_t refractory = pdMS_TO_TICKS(EMG_REFRACTORY_MS);
    emg_sample_t sample;

    for (;;) {
        if (xQueueReceive(sample_queue, &sample, portMAX_DELAY) != pdPASS)
            continue;

        const TickType_t now = xTaskGetTickCount();
        if (!armed && sample.envelope <= EMG_RELEASE_THRESHOLD &&
            (TickType_t)(now - last_activation) >= refractory)
            armed = true;

        if (armed && sample.envelope >= EMG_THRESHOLD) {
            hand_closed = !hand_closed;
            armed = false;
            last_activation = now;
            if (hand_closed) close_hand(pulse_us);
            else open_hand(pulse_us);

            taskENTER_CRITICAL();
            app_status.hand_closed = hand_closed;
            app_status.activation_count++;
            taskEXIT_CRITICAL();
        }
    }
}

static void log_task(void *unused) {
    (void)unused;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_activation_count = 0u;
    bool announced_calibration = false;

    for (;;) {
        app_status_t snapshot;
        taskENTER_CRITICAL();
        snapshot = app_status;
        taskEXIT_CRITICAL();

        if (!snapshot.calibrated) {
            if (!announced_calibration) {
                printf("Mantené el músculo relajado: calibrando nivel de reposo.\n");
                announced_calibration = true;
            }
        } else {
            if (last_activation_count != snapshot.activation_count) {
                printf("Activacion EMG: ADC=%u, envolvente=%lu, mano=%s\n",
                snapshot.sample.adc,
                (unsigned long)snapshot.sample.envelope,
                snapshot.hand_closed ? "cerrada" : "abierta");
                last_activation_count = snapshot.activation_count;
            }
            printf("ADC=%u, reposo=%u, envolvente=%lu, mano=%s\n",
            snapshot.sample.adc, snapshot.sample.baseline,
            (unsigned long)snapshot.sample.envelope,
            snapshot.hand_closed ? "cerrada" : "abierta");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(EMG_LOG_INTERVAL_MS));
    }
}

int main(void) {
    stdio_init_all();

    uint32_t pulse_us[SERVO_COUNT];
    initialize_pwm(pulse_us);

    adc_init();
    adc_gpio_init(EMG_ADC_GPIO);
    adc_select_input(EMG_ADC_INPUT);

    sample_queue = xQueueCreate(1u, sizeof(emg_sample_t));
    if (sample_queue == NULL) {
        while (true) tight_loop_contents();
    }

    if (xTaskCreate(emg_task, "EMG", 512u, NULL, 3u, NULL) != pdPASS ||
        xTaskCreate(control_task, "Control", 512u, NULL, 2u, NULL) != pdPASS ||
        xTaskCreate(log_task, "Log", 768u, NULL, 1u, NULL) != pdPASS) {
        while (true) tight_loop_contents();
    }

    vTaskStartScheduler();
    while (true) tight_loop_contents();
}
