#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

// Configuracion de cada servo: pin GPIO y pulsos minimos, neutros y maximos en microsegundos.
typedef struct {
    uint gpio;
    uint32_t min_pulse_us;
    uint32_t neutral_pulse_us;
    uint32_t max_pulse_us;
} servo_config_t;
#define SERVO_COUNT 6
static const servo_config_t servos[SERVO_COUNT] = {
    {0, 500u, 1510u, 2500u},   // Pulgar
    {2, 500u, 1510u, 2500u},   // Índice
    {4, 500u, 1510u, 2500u},   // Medio
    {6, 500u, 1510u, 2500u},   // Anular
    {8, 500u, 1510u, 2500u},   // Meñique
    {10, 500u, 1510u, 2500u}   // Muñeca
};

// Servo de muñeca (controlado por señales de giro horario/antihorario)
#define SERVO_MUNECA_IDX 5  // Índice del servo de muñeca
#define SERVO_PULGAR_IDX 0  // Índice del pulgar
#define SERVO_INDICE_IDX 1  // Índice del Índice

#define SERVO_FREQUENCY_HZ 50u
#define SERVO_FRAME_US (1000000u / SERVO_FREQUENCY_HZ)
#define SERVO_SWEEP_DELAY_MS 20

// ADS1115 conectado al Pico va I2C
#define ADS1115_I2C i2c0
#define ADS1115_SDA_PIN 16
#define ADS1115_SCL_PIN 17
#define ADS1115_ADDR 0x48  // Dirección I2C del ADS1115 (ADDR a GND)
#define ADS1115_CONVERSION_REGISTER 0x00
#define ADS1115_CONFIG_REGISTER 0x01

// Multiplexor de 4 canales con salida a ADS1115 AIN0.
// S0-S2 seleccionan el canal. No se usa S3 porque solo hay 4 entradas activas.
#define MUX_PIN_S0 18  // GP18
#define MUX_PIN_S1 19  // GP19
#define MUX_PIN_S2 20  // GP20
#define MUX_CHANNEL_COUNT 4

// Variables de estado
static bool modo_5_servos = true;  // true = 5 servos, false = 2 servos (pulgar+indice)

// Estructura de datos de los 4 canales EMG del ADS1115
typedef struct {
    uint16_t canal_excitAR;      // Canal 0 ADS1115: señal para excitar servos
    uint16_t canal_horario;     // Canal 1 ADS1115: señal giro horario muñeca
    uint16_t canal_antihorario; // Canal 2 ADS1115: señal giro antihorario muñeca
    uint16_t canal_modo;        // Canal 3 ADS1115: señal cambio de modo
} senales_control_t;

// Par�metros PWM para los servos
static inline uint32_t servo_pulse_to_level(uint32_t pulse_us) {
    return pulse_us;
}

// Funciones para leer el ADS1115
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

static void mux_select_channel(uint8_t channel) {
    gpio_put(MUX_PIN_S0, channel & 0x1);
    gpio_put(MUX_PIN_S1, (channel >> 1) & 0x1);
    gpio_put(MUX_PIN_S2, (channel >> 2) & 0x1);
}

static bool ads1115_read_mux_channel(uint8_t addr, uint8_t mux_channel, uint16_t *raw) {
    mux_select_channel(mux_channel);
    sleep_ms(1);

    uint16_t config = 0xC583; // AIN0-GND, 128SPS, +/-4.096V
    if (!ads1115_write_config(addr, config)) return false;
    sleep_ms(10);

    int16_t raw_val;
    if (!ads1115_read_conversion(addr, &raw_val)) return false;

    *raw = (uint16_t)(raw_val & 0xFFFF);
    return true;
}

// Leer los 4 canales del multiplexor conectados a la entrada AIN0 del ADS1115
static bool leer_senales_ads1115(senales_control_t *datos) {
    for (uint8_t canal = 0; canal < MUX_CHANNEL_COUNT; canal++) {
        uint16_t valor_raw;
        if (!ads1115_read_mux_channel(ADS1115_ADDR, canal, &valor_raw)) {
            return false;
        }
        
        switch (canal) {
            case 0: datos->canal_excitAR = valor_raw; break;
            case 1: datos->canal_horario = valor_raw; break;
            case 2: datos->canal_antihorario = valor_raw; break;
            case 3: datos->canal_modo = valor_raw; break;
        }
    }
    return true;
}

// Procesar cambio de modo con detector de pulso de 500ms
// Se detecta cuando el canal 3 supera un umbral (señal presente)
static void procesar_cambio_modo(uint16_t valor_canal_modo) {
    static uint32_t tiempo_pulso_inicio = 0;
    static bool en_pulso = false;
    static bool ultimo_estado = false;
    
    uint32_t tiempo_actual = to_ms_since_boot(get_absolute_time());
    
    // Umbral para detectar señal activa (aprox 1V = 32768 counts del ADC)
    bool seal_activa = valor_canal_modo > 30000;
    
    if (seal_activa && !ultimo_estado && !en_pulso) {
        // Inicio de pulso detectado
        tiempo_pulso_inicio = tiempo_actual;
        en_pulso = true;
    } else if (!seal_activa && en_pulso) {
        // Fin de pulso - verificar duración
        uint32_t duracion_pulso = tiempo_actual - tiempo_pulso_inicio;
        
        // Si el pulso dura entre 400ms y 600ms, cambiar modo
        if (duracion_pulso >= 400 && duracion_pulso <= 600) {
            modo_5_servos = !modo_5_servos;
            printf("Cambio de modo: %s\n", modo_5_servos ? "5 servos (dedos)" : "2 servos (pulgar+indice)");
        }
        
        en_pulso = false;
    }
    
    ultimo_estado = seal_activa;
}

// Mover servo de muñeca en sentido horario
static void mover_servo_muneca_horario(uint32_t *pulse_us) {
    if (*pulse_us < servos[SERVO_MUNECA_IDX].max_pulse_us) {
        *pulse_us += 20;  // Incremento por paso
        if (*pulse_us > servos[SERVO_MUNECA_IDX].max_pulse_us) {
            *pulse_us = servos[SERVO_MUNECA_IDX].max_pulse_us;
        }
    }
}

// Mover servo de muñeca en sentido antihorario
static void mover_servo_muneca_antihorario(uint32_t *pulse_us) {
    if (*pulse_us > servos[SERVO_MUNECA_IDX].min_pulse_us) {
        *pulse_us -= 20;  // Decremento por paso
        if (*pulse_us < servos[SERVO_MUNECA_IDX].min_pulse_us) {
            *pulse_us = servos[SERVO_MUNECA_IDX].min_pulse_us;
        }
    }
}

#define EMG_THRESHOLD 2000u // Ajustar según calibración del ADS1115 y el sensor EMG

static const uint32_t finger_open_angle[5] = {170u, 170u, 170u, 170u, 110u};
static const uint32_t finger_close_angle[5] = {0u, 0u, 0u, 0u, 0u};

static uint32_t map_angle_to_pulse(uint32_t servo_index, uint32_t angle_deg) {
    if (angle_deg > 180u) angle_deg = 180u;
    uint32_t min_pulse = servos[servo_index].min_pulse_us;
    uint32_t max_pulse = servos[servo_index].max_pulse_us;
    return min_pulse + ((angle_deg * (max_pulse - min_pulse)) / 180u);
}

static void set_servo_angle(uint32_t servo_index, uint32_t angle_deg, uint32_t *pulse_us) {
    pulse_us[servo_index] = map_angle_to_pulse(servo_index, angle_deg);
    pwm_set_gpio_level(servos[servo_index].gpio, servo_pulse_to_level(pulse_us[servo_index]));
}

static void open_hand(uint32_t *pulse_us) {
    for (uint i = 0; i < 5; ++i) {
        set_servo_angle(i, finger_open_angle[i], pulse_us);
    }
}

static void close_hand(uint32_t *pulse_us) {
    for (uint i = 0; i < 5; ++i) {
        set_servo_angle(i, finger_close_angle[i], pulse_us);
    }
}

static void open_two_servos(uint32_t *pulse_us) {
    set_servo_angle(SERVO_PULGAR_IDX, finger_open_angle[SERVO_PULGAR_IDX], pulse_us);
    set_servo_angle(SERVO_INDICE_IDX, finger_open_angle[SERVO_INDICE_IDX], pulse_us);
    for (uint i = 0; i < SERVO_COUNT; ++i) {
        if (i == SERVO_MUNECA_IDX) continue;
        if (i != SERVO_PULGAR_IDX && i != SERVO_INDICE_IDX) {
            pulse_us[i] = servos[i].neutral_pulse_us;
            pwm_set_gpio_level(servos[i].gpio, servo_pulse_to_level(pulse_us[i]));
        }
    }
}

static void close_two_servos(uint32_t *pulse_us) {
    set_servo_angle(SERVO_PULGAR_IDX, finger_close_angle[SERVO_PULGAR_IDX], pulse_us);
    set_servo_angle(SERVO_INDICE_IDX, finger_close_angle[SERVO_INDICE_IDX], pulse_us);
    for (uint i = 0; i < SERVO_COUNT; ++i) {
        if (i == SERVO_MUNECA_IDX) continue;
        if (i != SERVO_PULGAR_IDX && i != SERVO_INDICE_IDX) {
            pulse_us[i] = servos[i].neutral_pulse_us;
            pwm_set_gpio_level(servos[i].gpio, servo_pulse_to_level(pulse_us[i]));
        }
    }
}

// Verificar si hay señal activa en el canal (umbral)
static bool señal_activa(uint16_t valor) {
    return valor > 30000;  // Umbral ~1.5V
}

int main() {
    stdio_init_all();

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));

    uint slice_nums[SERVO_COUNT];
    uint32_t pulse_us[SERVO_COUNT];

    pwm_config config = pwm_get_default_config();

    // Usar un reloj PWM de 1 MHz para que la anchura de pulso coincida con microsegundos.
    float clk_hz = (float)clock_get_hz(clk_sys);
    float clock_divider = clk_hz / 1000000.0f;
    if (clock_divider < 1.0f) {
        clock_divider = 1.0f;
    }
    pwm_config_set_clkdiv(&config, clock_divider);
    pwm_config_set_wrap(&config, SERVO_FRAME_US - 1);

    // Inicializar el bus I2C para el ADS1115
    i2c_init(ADS1115_I2C, 100000);
    gpio_set_function(ADS1115_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ADS1115_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ADS1115_SDA_PIN);
    gpio_pull_up(ADS1115_SCL_PIN);

    // Inicializar pines del multiplexor
    gpio_init(MUX_PIN_S0);
    gpio_init(MUX_PIN_S1);
    gpio_init(MUX_PIN_S2);
    gpio_set_dir(MUX_PIN_S0, GPIO_OUT);
    gpio_set_dir(MUX_PIN_S1, GPIO_OUT);
    gpio_set_dir(MUX_PIN_S2, GPIO_OUT);
    mux_select_channel(0);

    // Inicializar servos PWM
    for (uint i = 0; i < SERVO_COUNT; ++i) {
        gpio_set_function(servos[i].gpio, GPIO_FUNC_PWM);
        slice_nums[i] = pwm_gpio_to_slice_num(servos[i].gpio);
        pwm_init(slice_nums[i], &config, true);
        pulse_us[i] = servos[i].neutral_pulse_us;
        pwm_set_gpio_level(servos[i].gpio, servo_pulse_to_level(pulse_us[i]));
    }

    printf("Servo PWM started on %u channels at %u Hz\n", SERVO_COUNT, SERVO_FREQUENCY_HZ);
    printf("ADS1115 initialized on I2C0 (GPIO %d=SDA, %d=SCL) at address 0x%02X\n", 
    ADS1115_SDA_PIN, ADS1115_SCL_PIN, ADS1115_ADDR);
    printf("MUX select pins: S0=%d, S1=%d, S2=%d\n", MUX_PIN_S0, MUX_PIN_S1, MUX_PIN_S2);
    printf("Modo actual: %s\n", modo_5_servos ? "5 servos (dedos)" : "2 servos (pulgar+indice)");
    printf("Canales MUX -> ADS1115 AIN0: 0=Excitar, 1=Horario Muneca, 2=Antihorario Muneca, 3=Cambio Modo\n");

    uint32_t cycle_count = 0;
    senales_control_t senales_actuales = {0, 0, 0, 0};

    while (true) {
        // Leer los 4 canales del ADS1115
        if (leer_senales_ads1115(&senales_actuales)) {
            // Procesar cambio de modo (canal 3)
            procesar_cambio_modo(senales_actuales.canal_modo);
            
            // Procesar señales de giro de muñeca (canales 1 y 2)
            if (señal_activa(senales_actuales.canal_horario)) {
                mover_servo_muneca_horario(&pulse_us[SERVO_MUNECA_IDX]);
                pwm_set_gpio_level(servos[SERVO_MUNECA_IDX].gpio, servo_pulse_to_level(pulse_us[SERVO_MUNECA_IDX]));
                if (cycle_count % 50 == 0) {
                    printf("Muneca giro horario: pulso %u us\n", pulse_us[SERVO_MUNECA_IDX]);
                }
            }
            if (señal_activa(senales_actuales.canal_antihorario)) {
                mover_servo_muneca_antihorario(&pulse_us[SERVO_MUNECA_IDX]);
                pwm_set_gpio_level(servos[SERVO_MUNECA_IDX].gpio, servo_pulse_to_level(pulse_us[SERVO_MUNECA_IDX]));
                if (cycle_count % 50 == 0) {
                    printf("Muneca giro antihorario: pulso %u us\n", pulse_us[SERVO_MUNECA_IDX]);
                }
            }
            
            // Procesar señal de excitar (canal 0) según el modo actual y el umbral EMG
            bool emg_activa = senales_actuales.canal_excitAR > EMG_THRESHOLD;

            if (modo_5_servos) {
                if (emg_activa) {
                    close_hand(pulse_us);
                } else {
                    open_hand(pulse_us);
                }
                if (cycle_count % 50 == 0) {
                    printf("Modo 5 servos: EMG=%u -> %s\n", senales_actuales.canal_excitAR, emg_activa ? "cerrar" : "abrir");
                }
            } else {
                if (emg_activa) {
                    close_two_servos(pulse_us);
                } else {
                    open_two_servos(pulse_us);
                }
                if (cycle_count % 50 == 0) {
                    printf("Modo 2 servos (pulgar+indice): EMG=%u -> %s\n", senales_actuales.canal_excitAR, emg_activa ? "cerrar" : "abrir");
                }
            }
        } else {
            printf("ADS1115 read error\n");
        }

        cycle_count++;
        sleep_ms(SERVO_SWEEP_DELAY_MS);
    }
}