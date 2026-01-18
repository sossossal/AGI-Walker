/*
 * OpenNeuro ESP32 Demo - Main Firmware
 *
 * 功能: 通过Zenoh-Pico发送传感器数据到PC
 * 硬件: ESP32-S3 DevKit
 * 依赖: Zenoh-Pico, ESP-IDF 5.1+
 */

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>


#include "zenoh-pico.h"

// 配置参数 (通过menuconfig设置)
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD
#define ZENOH_ROUTER CONFIG_ZENOH_ROUTER_IP
#define ZENOH_PORT CONFIG_ZENOH_ROUTER_PORT

// Topic定义
#define TOPIC_IMU "/robot/sensor/imu"
#define TOPIC_CMD "/robot/control/servo"

static const char *TAG = "openneuro_esp32";

// Zenoh会话
static z_owned_session_t z_session;
static z_owned_publisher_t z_pub;
static z_owned_subscriber_t z_sub;

// WiFi事件处理
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGI(TAG, "WiFi断开，尝试重连...");
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "获得IP:" IPSTR, IP2STR(&event->ip_info.ip));
  }
}

// 初始化WiFi
void wifi_init_sta(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
      &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASS,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "WiFi初始化完成，连接到: %s", WIFI_SSID);
}

// Zenoh订阅回调 - 接收控制命令
void zenoh_subscriber_callback(const z_sample_t *sample, void *arg) {
  z_owned_string_t keystr = z_keyexpr_to_string(sample->keyexpr);

  // 解析payload
  char payload_str[256];
  memcpy(payload_str, sample->payload.start, sample->payload.len);
  payload_str[sample->payload.len] = '\0';

  ESP_LOGI(TAG, "收到Zenoh消息 [%s]: %s", z_string_data(z_string_loan(&keystr)),
           payload_str);

  // TODO: 解析并执行控制命令 (如驱动舵机)

  z_string_drop(z_move(keystr));
}

// 初始化Zenoh
int zenoh_init(void) {
  // 等待WiFi连接
  ESP_LOGI(TAG, "等待WiFi连接...");
  vTaskDelay(pdMS_TO_TICKS(5000));

  // Zenoh配置
  z_owned_config_t config = z_config_default();
  if (!z_config_check(&config)) {
    ESP_LOGE(TAG, "Zenoh配置创建失败");
    return -1;
  }

  // 设置为client模式
  zp_config_insert(z_config_loan(&config), Z_CONFIG_MODE_KEY,
                   z_string_make("client"));

  // 设置Router端点
  char endpoint[64];
  snprintf(endpoint, sizeof(endpoint), "tcp/%s:%d", ZENOH_ROUTER, ZENOH_PORT);
  zp_config_insert(z_config_loan(&config), Z_CONFIG_CONNECT_KEY,
                   z_string_make(endpoint));

  ESP_LOGI(TAG, "连接到Zenoh Router: %s", endpoint);

  // 打开Zenoh会话
  z_session = z_open(z_move(config));
  if (!z_session_check(&z_session)) {
    ESP_LOGE(TAG, "Zenoh会话打开失败");
    return -1;
  }

  ESP_LOGI(TAG, "Zenoh会话已建立");

  // 创建发布器
  z_keyexpr_t ke_pub = z_keyexpr(TOPIC_IMU);
  z_pub = z_declare_publisher(z_session_loan(&z_session), ke_pub, NULL);
  if (!z_publisher_check(&z_pub)) {
    ESP_LOGE(TAG, "Zenoh发布器创建失败");
    return -1;
  }

  ESP_LOGI(TAG, "Zenoh发布器已创建: %s", TOPIC_IMU);

  // 创建订阅器
  z_owned_closure_sample_t callback =
      z_closure(zenoh_subscriber_callback, NULL, NULL);
  z_keyexpr_t ke_sub = z_keyexpr(TOPIC_CMD);
  z_sub = z_declare_subscriber(z_session_loan(&z_session), ke_sub,
                               z_move(callback), NULL);
  if (!z_subscriber_check(&z_sub)) {
    ESP_LOGE(TAG, "Zenoh订阅器创建失败");
    return -1;
  }

  ESP_LOGI(TAG, "Zenoh订阅器已创建: %s", TOPIC_CMD);

  return 0;
}

// 读取模拟IMU数据
void read_imu_data(float *accel_x, float *accel_y, float *accel_z,
                   float *gyro_x, float *gyro_y, float *gyro_z) {
  // 模拟数据 (实际应用中替换为真实传感器读取)
  static float angle = 0.0f;
  angle += 0.1f;

  *accel_x = sinf(angle) * 9.8f; // m/s²
  *accel_y = cosf(angle) * 9.8f;
  *accel_z = 9.8f;

  *gyro_x = sinf(angle * 2) * 0.5f; // rad/s
  *gyro_y = cosf(angle * 2) * 0.5f;
  *gyro_z = 0.1f;
}

// Zenoh发布任务
void zenoh_publish_task(void *pvParameters) {
  char payload[256];
  int seq = 0;

  ESP_LOGI(TAG, "开始发布传感器数据 @ 100Hz");

  while (1) {
    // 读取传感器
    float ax, ay, az, gx, gy, gz;
    read_imu_data(&ax, &ay, &az, &gx, &gy, &gz);

    // 获取时间戳
    int64_t timestamp = esp_timer_get_time(); // 微秒

    // 构造JSON payload
    snprintf(payload, sizeof(payload),
             "{\"seq\":%d,\"time\":%lld,\"accel\":[%.3f,%.3f,%.3f],\"gyro\":[%."
             "3f,%.3f,%.3f]}",
             seq++, timestamp, ax, ay, az, gx, gy, gz);

    // 发布到Zenoh
    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_TEXT_JSON, NULL);

    z_publisher_put(z_publisher_loan(&z_pub), (const uint8_t *)payload,
                    strlen(payload), &options);

    // 每秒打印一次统计
    if (seq % 100 == 0) {
      ESP_LOGI(TAG, "已发送 %d 条消息", seq);
    }

    // 100Hz = 10ms周期
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, " OpenNeuro ESP32 Demo");
  ESP_LOGI(TAG, " Version: 1.0.0");
  ESP_LOGI(TAG, "========================================");

  // 初始化NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // 初始化WiFi
  wifi_init_sta();

  // 等待WiFi连接
  vTaskDelay(pdMS_TO_TICKS(10000));

  // 初始化Zenoh
  if (zenoh_init() != 0) {
    ESP_LOGE(TAG, "Zenoh初始化失败，重启中...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_restart();
  }

  // 创建发布任务
  xTaskCreate(zenoh_publish_task, "zenoh_pub", 4096, NULL, 5, NULL);

  ESP_LOGI(TAG, "系统启动完成，开始运行");
}
