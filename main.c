#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"

// ---------------- Configuración (CAMBIAR) ----------------
#define WIFI_SSID        "NOMBRE_RED"
#define WIFI_PASS        "CONTRASENA_RED"
#define MQTT_URI         "mqtt://192.168.0.22:1883"   // IP real de la Raspberry Pi
#define MQTT_TOPIC       "seccion10/equipo01"
#define SEC_EQ           "equipo01"

// ---------------- Pines ----------------
#define PIN_RC522_SS     5
#define PIN_RC522_SCK    18
#define PIN_RC522_MOSI   23
#define PIN_RC522_MISO   19
#define PIN_RC522_RST    22
#define PIN_SERVO        13
#define PIN_LED          26
#define MQ2_ADC_CHANNEL  ADC_CHANNEL_6               // GPIO34

// ---------------- Parámetros ----------------
#define INTERVALO_GAS_MS     5000
#define TIEMPO_ABIERTO_MS    5000
#define TIEMPO_LED_DENEGADO  3000
#define UMBRAL_HUMO_PPM      200

// UID de la tarjeta autorizada (tarjeta blanca) - CAMBIAR
static const uint8_t UID_AUTORIZADO[] = {0xAA, 0xBB, 0xCC, 0xDD};

// Calibración MQ-2 (reemplazar por las ecuaciones proporcionadas)
#define MQ2_RL_KOHM      5.0f
#define MQ2_R0_KOHM      10.0f
#define MQ2_VCC          5.0f
#define MQ2_DIVISOR      2.0f        // factor del divisor de voltaje en AO
#define MQ2_A            3616.1f
#define MQ2_B            -2.675f

#define millis() (uint32_t)(esp_timer_get_time() / 1000)

static const char *TAG = "RETO5";

// ---------------- Variables globales ----------------
esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_iniciado = false;

static mcpwm_cmpr_handle_t servo_cmpr = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;

static volatile bool tarjeta_pendiente = false;
static uint8_t uid_leido[10];
static volatile uint8_t uid_len = 0;

static bool puerta_abierta = false;
static uint32_t t_cierre = 0;
static uint32_t t_led_denegado = 0;
static bool led_denegado_activo = false;
static int32_t humo_ppm = 0;

// ---------------- MQTT ----------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI("MQTT", "Conectado exitosamente al broker en la Raspberry Pi");
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        ESP_LOGW("MQTT", "Desconectado del broker MQTT");
    }
}

void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
}

static void mqtt_publicar(const char *payload) {
    if (mqtt_client != NULL) {
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
    }
    ESP_LOGI(TAG, "Publicado: %s", payload);
}

// ---------------- Wi-Fi ----------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW("WIFI", "Wi-Fi desconectado, reintentando...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI("WIFI", "Wi-Fi Conectado. Iniciando MQTT...");
        if (mqtt_client != NULL && !mqtt_iniciado) {
            esp_mqtt_client_start(mqtt_client);
            mqtt_iniciado = true;
        }
    }
}

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ---------------- Servomotor (MCPWM) ----------------
static void servo_init(void) {
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,          // 1 tick = 1 us
        .period_ticks = 20000,             // 20 ms = 50 Hz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t cmpr_cfg = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_cfg, &servo_cmpr));

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = PIN_SERVO };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, servo_cmpr, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static void servo_set_angulo(int grados) {
    // 0 grados = 500 us, 180 grados = 2500 us
    uint32_t pulso_us = 500 + (grados * 2000) / 180;
    mcpwm_comparator_set_compare_value(servo_cmpr, pulso_us);
}

// ---------------- LED y MQ-2 ----------------
static void led_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LED, 0);
}

static void adc_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MQ2_ADC_CHANNEL, &chan_cfg));
}

static int32_t leer_humo_ppm(void) {
    int raw = 0, suma = 0;
    for (int i = 0; i < 10; i++) {
        adc_oneshot_read(adc_handle, MQ2_ADC_CHANNEL, &raw);
        suma += raw;
    }
    float adc_prom = suma / 10.0f;
    float v_adc = adc_prom * 3.3f / 4095.0f;
    float v_sensor = v_adc * MQ2_DIVISOR;
    if (v_sensor < 0.01f) v_sensor = 0.01f;

    float rs = MQ2_RL_KOHM * (MQ2_VCC - v_sensor) / v_sensor;
    float ratio = rs / MQ2_R0_KOHM;
    float ppm = MQ2_A * powf(ratio, MQ2_B);
    if (ppm < 0) ppm = 0;
    return (int32_t)ppm;
}

// ---------------- RFID ----------------
static void on_picc_state_changed(void *arg, esp_event_base_t base,
                                  int32_t event_id, void *data) {
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    if (picc->state == RC522_PICC_STATE_ACTIVE && !tarjeta_pendiente) {
        uid_len = picc->uid.length;
        memcpy(uid_leido, picc->uid.value, picc->uid.length);
        tarjeta_pendiente = true;      // se procesa en el bucle principal
    }
}

static void rfid_init(void) {
    rc522_spi_config_t driver_cfg = {
        .host_id = SPI3_HOST,
        .bus_config = &(spi_bus_config_t){
            .miso_io_num = PIN_RC522_MISO,
            .mosi_io_num = PIN_RC522_MOSI,
            .sclk_io_num = PIN_RC522_SCK,
        },
        .dev_config = { .spics_io_num = PIN_RC522_SS },
        .rst_io_num = PIN_RC522_RST,
    };
    rc522_driver_handle_t driver;
    rc522_handle_t scanner;
    ESP_ERROR_CHECK(rc522_spi_create(&driver_cfg, &driver));
    ESP_ERROR_CHECK(rc522_driver_install(driver));
    rc522_config_t scanner_cfg = { .driver = driver };
    ESP_ERROR_CHECK(rc522_create(&scanner_cfg, &scanner));
    ESP_ERROR_CHECK(rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL));
    ESP_ERROR_CHECK(rc522_start(scanner));
}

static void procesar_tarjeta(void) {
    char log_uid[32] = "";
    for (int i = 0; i < uid_len; i++) {
        char b[4];
        snprintf(b, sizeof(b), "%02X ", uid_leido[i]);
        strncat(log_uid, b, sizeof(log_uid) - strlen(log_uid) - 1);
    }
    ESP_LOGI(TAG, "UID leido: %s", log_uid);

    bool autorizado = (uid_len == sizeof(UID_AUTORIZADO)) &&
                      (memcmp(uid_leido, UID_AUTORIZADO, uid_len) == 0);
    char payload[64];

    if (autorizado) {
        snprintf(payload, sizeof(payload), "acceso/%s/PERMITIDO", SEC_EQ);
        mqtt_publicar(payload);
        if (!puerta_abierta) {
            servo_set_angulo(90);
            puerta_abierta = true;
            t_cierre = millis() + TIEMPO_ABIERTO_MS;
            snprintf(payload, sizeof(payload), "actuador/%s/ON", SEC_EQ);
            mqtt_publicar(payload);
        }
    } else {
        snprintf(payload, sizeof(payload), "acceso/%s/DENEGADO", SEC_EQ);
        mqtt_publicar(payload);
        led_denegado_activo = true;
        t_led_denegado = millis() + TIEMPO_LED_DENEGADO;
    }
}

// ---------------- Programa principal ----------------
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    led_init();
    servo_init();
    servo_set_angulo(0);
    adc_init();
    mqtt_init();        // antes de Wi-Fi, para que el cliente exista al obtener IP
    wifi_init_sta();
    rfid_init();

    uint32_t t_gas = millis();

    while (1) {
        uint32_t ahora = millis();

        // 1) Tarjeta detectada por el lector RFID
        if (tarjeta_pendiente) {
            procesar_tarjeta();
            tarjeta_pendiente = false;
        }

        // 2) Cierre de la puerta tras 5 s (sin bloquear)
        if (puerta_abierta && (int32_t)(ahora - t_cierre) >= 0) {
            servo_set_angulo(0);
            puerta_abierta = false;
            char payload[64];
            snprintf(payload, sizeof(payload), "actuador/%s/OFF", SEC_EQ);
            mqtt_publicar(payload);
        }

        // 3) Lectura del MQ-2 cada 5 s (sin vTaskDelay de 5 s)
        if ((ahora - t_gas) >= INTERVALO_GAS_MS) {
            t_gas = ahora;
            humo_ppm = leer_humo_ppm();
            char payload[64];
            snprintf(payload, sizeof(payload), "humo/%s/%ld", SEC_EQ, (long)humo_ppm);
            mqtt_publicar(payload);
        }

        // 4) LED rojo: humo > 200 PPM o acceso denegado reciente
        if (led_denegado_activo && (int32_t)(ahora - t_led_denegado) >= 0) {
            led_denegado_activo = false;
        }
        gpio_set_level(PIN_LED, (humo_ppm > UMBRAL_HUMO_PPM || led_denegado_activo) ? 1 : 0);

        // Solo cede CPU 10 ms al planificador (no es la pausa de 5 s)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
