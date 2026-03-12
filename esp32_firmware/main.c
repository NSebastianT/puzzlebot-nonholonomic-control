// ============================================================
// micro-ROS ESP32 — PuzzleBot motor driver
// Suscribe:  /cmd_vel        (geometry_msgs/Twist)
// Publica:   /VelocityEncL   (std_msgs/Float32)
//            /VelocityEncR   (std_msgs/Float32)
// Pines:
//   Motor IZQ : ENA=25, IN1=26, IN2=27
//   Motor DER : ENB=14, IN3=12, IN4=13
//   Enc   IZQ : A=34,  B=35
//   Enc   DER : A=33,  B=32
// ============================================================

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

static const char *TAG = "puzzlebot";

#define WIFI_SSID           CONFIG_ESP_WIFI_SSID
#define WIFI_PASS           CONFIG_ESP_WIFI_PASSWORD
#define AGENT_IP            CONFIG_MICRO_ROS_AGENT_IP
#define AGENT_PORT          CONFIG_MICRO_ROS_AGENT_PORT
#define WIFI_CONNECTED_BIT  BIT0

static EventGroupHandle_t wifi_event_group;

// ── Pines ─────────────────────────────────────────────────
#define PIN_ENA  25
#define PIN_IN1  26
#define PIN_IN2  27
#define PIN_ENB  14
#define PIN_IN3  12
#define PIN_IN4  13
#define ENC_L_A  34
#define ENC_L_B  35
#define ENC_R_A  33
#define ENC_R_B  32

// ── PWM ───────────────────────────────────────────────────
#define PWM_FREQ   25000
#define PWM_RES    LEDC_TIMER_8_BIT

// ── Robot ─────────────────────────────────────────────────
#define WHEEL_BASE    0.19f
#define WHEEL_RADIUS  0.05f
#define CPR           1980.0f

// ── PI ────────────────────────────────────────────────────
#define PWM_FF     115.0f
#define KP           1.0f
#define KI           6.0f
#define ITERM_MIN -120.0f
#define ITERM_MAX  120.0f
#define CTRL_MS      100
#define RPM_ALPHA    0.7f

// ── Estado ────────────────────────────────────────────────
static volatile int32_t enc_L = 0, enc_R = 0;
static volatile uint8_t last_ab_L = 0, last_ab_R = 0;
static float iterm_L = 0, iterm_R = 0;
static float rpm_filt_L = 0, rpm_filt_R = 0;
static volatile float target_wl = 0.0f;
static volatile float target_wr = 0.0f;

static geometry_msgs__msg__Twist  cmd_vel_msg;
static std_msgs__msg__Float32     enc_L_msg, enc_R_msg;

static const int8_t qem[16] = {
    0,-1,+1, 0,
   +1, 0, 0,-1,
   -1, 0, 0,+1,
    0,+1,-1, 0
};

// ── ISRs encoder ──────────────────────────────────────────
static void IRAM_ATTR isr_enc_L(void *arg) {
    uint8_t ab = ((uint8_t)gpio_get_level(ENC_L_A) << 1)
               | (uint8_t)gpio_get_level(ENC_L_B);
    last_ab_L = ((last_ab_L << 2) | ab) & 0x0F;
    enc_L += qem[last_ab_L];
}

static void IRAM_ATTR isr_enc_R(void *arg) {
    uint8_t ab = ((uint8_t)gpio_get_level(ENC_R_A) << 1)
               | (uint8_t)gpio_get_level(ENC_R_B);
    last_ab_R = ((last_ab_R << 2) | ab) & 0x0F;
    enc_R += qem[last_ab_R];
}

// ── Motor helpers ─────────────────────────────────────────
static void motor_L(int pwm) {
    if (pwm >= 0) {
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
    } else {
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 1);
        pwm = -pwm;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                  (uint32_t)(pwm > 255 ? 255 : pwm));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void motor_R(int pwm) {
    if (pwm >= 0) {
        gpio_set_level(PIN_IN3, 0);
        gpio_set_level(PIN_IN4, 1);
    } else {
        gpio_set_level(PIN_IN3, 1);
        gpio_set_level(PIN_IN4, 0);
        pwm = -pwm;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,
                  (uint32_t)(pwm > 255 ? 255 : pwm));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static void stop_all(void) {
    gpio_set_level(PIN_IN1, 0);
    gpio_set_level(PIN_IN2, 0);
    gpio_set_level(PIN_IN3, 0);
    gpio_set_level(PIN_IN4, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// ── PI ────────────────────────────────────────────────────
static float pi_step(float tgt, float meas, float *it, float dt) {
    float err = tgt - meas;
    *it += err * dt;
    if (*it < ITERM_MIN) { *it = ITERM_MIN; }
    if (*it > ITERM_MAX) { *it = ITERM_MAX; }
    float u = PWM_FF + KP * err + KI * (*it);
    if (u < 0.0f)   { u = 0.0f; }
    if (u > 255.0f) { u = 255.0f; }
    return u;
}

// ── Callback cmd_vel ──────────────────────────────────────
static void cmd_vel_cb(const void *msg_in) {
    const geometry_msgs__msg__Twist *m =
        (const geometry_msgs__msg__Twist *)msg_in;
    float v = (float)m->linear.x;
    float w = (float)m->angular.z;
    target_wl = (2.0f * v - w * WHEEL_BASE) / (2.0f * WHEEL_RADIUS);
    target_wr = (2.0f * v + w * WHEEL_BASE) / (2.0f * WHEEL_RADIUS);
}

// ── Hardware init ─────────────────────────────────────────
static void hw_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<PIN_IN1)|(1ULL<<PIN_IN2)|
                        (1ULL<<PIN_IN3)|(1ULL<<PIN_IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch0 = {
        .gpio_num   = PIN_ENA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch0);

    ledc_channel_config_t ch1 = {
        .gpio_num   = PIN_ENB,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch1);
    stop_all();

    // Encoders IZQ: GPIO 34/35 son input-only, sin pullup
    gpio_config_t ei = {
        .pin_bit_mask = (1ULL<<ENC_L_A)|(1ULL<<ENC_L_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&ei);

    // Encoders DER: GPIO 32/33 soportan pullup
    gpio_config_t ei2 = {
        .pin_bit_mask = (1ULL<<ENC_R_A)|(1ULL<<ENC_R_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&ei2);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENC_L_A, isr_enc_L, NULL);
    gpio_isr_handler_add(ENC_L_B, isr_enc_L, NULL);
    gpio_isr_handler_add(ENC_R_A, isr_enc_R, NULL);
    gpio_isr_handler_add(ENC_R_B, isr_enc_R, NULL);

    ESP_LOGI(TAG, "Hardware listo.");
}

// ── WiFi ──────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void) {
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               wifi_event_handler, NULL);

    wifi_config_t wc = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi conectado.");
}

// ── micro-ROS task ────────────────────────────────────────
static void microros_task(void *arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Configura opciones con IP/puerto del agente UDP
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(AGENT_IP, AGENT_PORT, rmw_options);

    rclc_support_t support;
    rclc_support_init_with_options(&support, 0, NULL,
                                   &init_options, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "puzzlebot_driver", "", &support);

    // Suscriptor /cmd_vel
    rcl_subscription_t sub_cmd;
    rclc_subscription_init_default(&sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    // Publicadores encoders
    rcl_publisher_t pub_L, pub_R;
    rclc_publisher_init_default(&pub_L, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/VelocityEncL");
    rclc_publisher_init_default(&pub_R, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/VelocityEncR");

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd,
        &cmd_vel_msg, &cmd_vel_cb, ON_NEW_DATA);

    ESP_LOGI(TAG, "micro-ROS listo. Esperando /cmd_vel...");

    int32_t prev_L = 0, prev_R = 0;
    TickType_t t_prev = xTaskGetTickCount();

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        TickType_t now = xTaskGetTickCount();
        uint32_t elapsed = (now - t_prev) * portTICK_PERIOD_MS;
        if (elapsed < CTRL_MS) {
            continue;
        }

        float dt = (float)elapsed / 1000.0f;
        t_prev = now;

        portDISABLE_INTERRUPTS();
        int32_t cL = enc_L;
        int32_t cR = enc_R;
        portENABLE_INTERRUPTS();

        int32_t dL = cL - prev_L;
        int32_t dR = cR - prev_R;
        prev_L = cL;
        prev_R = cR;

        float rpm_L = ((float)dL / CPR) * (60.0f / dt);
        float rpm_R = ((float)dR / CPR) * (60.0f / dt);

        rpm_filt_L = RPM_ALPHA * rpm_filt_L + (1.0f - RPM_ALPHA) * rpm_L;
        rpm_filt_R = RPM_ALPHA * rpm_filt_R + (1.0f - RPM_ALPHA) * rpm_R;

        // Publica velocidad angular (rad/s)
        enc_L_msg.data = rpm_filt_L * 2.0f * (float)M_PI / 60.0f;
        enc_R_msg.data = rpm_filt_R * 2.0f * (float)M_PI / 60.0f;
        rcl_publish(&pub_L, &enc_L_msg, NULL);
        rcl_publish(&pub_R, &enc_R_msg, NULL);

        // Convierte target rad/s → RPM
        float tgt_rpm_L = target_wl * 60.0f / (2.0f * (float)M_PI);
        float tgt_rpm_R = target_wr * 60.0f / (2.0f * (float)M_PI);

        if (fabsf(tgt_rpm_L) < 0.5f && fabsf(tgt_rpm_R) < 0.5f) {
            stop_all();
            iterm_L = 0;
            iterm_R = 0;
        } else {
            motor_L((int)pi_step(tgt_rpm_L, rpm_filt_L, &iterm_L, dt));
            motor_R((int)pi_step(tgt_rpm_R, rpm_filt_R, &iterm_R, dt));
        }
    }

    vTaskDelete(NULL);
}

// ── app_main ──────────────────────────────────────────────
void app_main(void) {
    nvs_flash_init();
    hw_init();
    wifi_init();
    xTaskCreate(microros_task, "microros_task", 8192, NULL, 5, NULL);
}
