#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

// Configuración de cada servo: pin GPIO y pulsos mínimos, neutros y máximos en microsegundos.
typedef struct {
    uint gpio;
    uint32_t min_pulse_us;
    uint32_t neutral_pulse_us;
    uint32_t max_pulse_us;
} servo_config_t;

#define SERVO_COUNT 5
static const servo_config_t servos[SERVO_COUNT] = {
    {0, 500u, 1510u, 2500u},
    {2, 500u, 1510u, 2500u},
    {4, 500u, 1510u, 2500u},
    {6, 500u, 1510u, 2500u},
    {8, 500u, 1510u, 2500u}
};
#define SERVO_FREQUENCY_HZ 50u
#define SERVO_FRAME_US (1000000u / SERVO_FREQUENCY_HZ)
#define SERVO_SWEEP_STEP_US 10u
#define SERVO_SWEEP_DELAY_MS 20

#define ADS1115_I2C i2c1
#define ADS1115_SDA_PIN 14
#define ADS1115_SCL_PIN 15
#define ADS1115_ADDR1 0x48
#define ADS1115_ADDR2 0x49
#define ADS1115_CONVERSION_REGISTER 0x00
#define ADS1115_CONFIG_REGISTER 0x01

// Parámetros PWM para los servos:
// - Frecuencia de 50 Hz
// - Posición 0° ≈ 500–600 µs
// - Posición neutra 90° ≈ 1500–1520 µs
// - Posición 180° ≈ 2400–2500 µs
static inline uint32_t servo_pulse_to_level(uint32_t pulse_us) {
    return pulse_us;
}

static bool ads1115_write_config(uint8_t addr, uint16_t config) {
    uint8_t buffer[3];
    buffer[0] = ADS1115_CONFIG_REGISTER;
    buffer[1] = (config >> 8) & 0xFF;
    buffer[2] = config & 0xFF;
    int ret = i2c_write_blocking(ADS1115_I2C, addr, buffer, sizeof(buffer), false);
    return ret == sizeof(buffer);
}

static bool ads1115_read_conversion(uint8_t addr, int16_t *value) {
    uint8_t reg = ADS1115_CONVERSION_REGISTER;
    int ret = i2c_write_blocking(ADS1115_I2C, addr, &reg, 1, true);
    if (ret != 1) return false;

    uint8_t buffer[2];
    ret = i2c_read_blocking(ADS1115_I2C, addr, buffer, 2, false);
    if (ret != 2) return false;

    *value = (buffer[0] << 8) | buffer[1];
    return true;
}

static bool ads1115_read_channel(uint8_t addr, uint8_t channel, float *voltage) {
    uint16_t config;
    switch (channel) {
        case 0: config = 0xC583; break; // AIN0-GND
        case 1: config = 0xD583; break; // AIN1-GND
        case 2: config = 0xE583; break; // AIN2-GND
        case 3: config = 0xF583; break; // AIN3-GND
        default: return false;
    }
    if (!ads1115_write_config(addr, config)) return false;
    sleep_ms(10);

    int16_t raw;
    if (!ads1115_read_conversion(addr, &raw)) return false;

    *voltage = raw * 0.000125f; // 4.096V full scale, 32768 counts
    return true;
}

int main() {
    stdio_init_all();

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));

    uint slice_nums[SERVO_COUNT];
    uint32_t pulse_us[SERVO_COUNT];
    bool increasing[SERVO_COUNT];

    pwm_config config = pwm_get_default_config();

    // Usar un reloj PWM de 1 MHz para que la anchura de pulso coincida con microsegundos.
    float clk_hz = (float)clock_get_hz(clk_sys);
    float clock_divider = clk_hz / 1000000.0f;
    if (clock_divider < 1.0f) {
        clock_divider = 1.0f;
    }
    pwm_config_set_clkdiv(&config, clock_divider);
    pwm_config_set_wrap(&config, SERVO_FRAME_US - 1);

    // Inicializar el bus I2C y configurar los pines SDA/SCL con pull-up.
    i2c_init(ADS1115_I2C, 100000);
    gpio_set_function(ADS1115_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ADS1115_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ADS1115_SDA_PIN);
    gpio_pull_up(ADS1115_SCL_PIN);

    for (uint i = 0; i < SERVO_COUNT; ++i) {
        // Configurar cada pin GPIO como salida PWM para los servos.
        gpio_set_function(servos[i].gpio, GPIO_FUNC_PWM);
        slice_nums[i] = pwm_gpio_to_slice_num(servos[i].gpio);
        pwm_init(slice_nums[i], &config, true);
        pulse_us[i] = servos[i].neutral_pulse_us;
        increasing[i] = true;
        pwm_set_gpio_level(servos[i].gpio, servo_pulse_to_level(pulse_us[i]));
    }

    printf("Servo PWM started on %u channels at %u Hz\n", SERVO_COUNT, SERVO_FREQUENCY_HZ);
    printf("ADS1115 initialized on I2C1 (GPIO %d=SDA, %d=SCL) with addresses 0x%02X and 0x%02X\n", ADS1115_SDA_PIN, ADS1115_SCL_PIN, ADS1115_ADDR1, ADS1115_ADDR2);

    uint32_t cycle_count = 0;
    while (true) {
        // Leer el ADS1115 cada 50 ciclos y ajustar los servos según el voltaje de cada canal.
        if (++cycle_count >= 50) {
            cycle_count = 0;
            for (uint i = 0; i < SERVO_COUNT; ++i) {
                uint8_t addr = (i < 4) ? ADS1115_ADDR1 : ADS1115_ADDR2;
                uint8_t channel = (i < 4) ? i : 0;
                float voltage;
                if (ads1115_read_channel(addr, channel, &voltage)) {
                    // Calcular grados: 0V = 0°, 4.096V = 180°
                    float degrees = voltage * 180.0f / 4.096f;
                    // Calcular pulso: 500us = 0°, 2500us = 180°
                    uint32_t pulse = 500 + (uint32_t)(degrees * 2000.0f / 180.0f);
                    // Limitar el pulso entre min y max
                    if (pulse < servos[i].min_pulse_us) pulse = servos[i].min_pulse_us;
                    if (pulse > servos[i].max_pulse_us) pulse = servos[i].max_pulse_us;

                    pulse_us[i] = pulse;
                    pwm_set_gpio_level(servos[i].gpio, servo_pulse_to_level(pulse_us[i]));

                    printf("Servo %u (ADC 0x%02X AIN%u) = %.3f V -> %.1f grados -> pulso %u us\n", i, addr, channel, voltage, degrees, pulse);
                } else {
                    printf("ADS1115 read error on 0x%02X channel %u\n", addr, channel);
                }
            }
        }

        sleep_ms(SERVO_SWEEP_DELAY_MS);
    }
}
