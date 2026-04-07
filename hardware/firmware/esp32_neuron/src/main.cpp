/**
 * AGI-Walker ESP32 Neuron Firmware
 * 基于 OpenNeuro Zenoh-Pico 架构
 *
 * 功能:
 * - Zenoh-Pico 通信
 * - PWM 电机控制
 * - 传感器数据采集
 * - PTP 时间同步 (可选)
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <zenoh-pico.h>


// ==================== 配置 ====================

// WiFi 配置
const char *WIFI_SSID = "YourSSID";
const char *WIFI_PASSWORD = "YourPassword";

// Zenoh 配置
const char *ZENOH_ROUTER = "tcp/192.168.1.100:7447"; // Ganglion/Cortex 地址
const char *ZENOH_CMD_KEY = "rt/neuron/cmd";
const char *ZENOH_STATE_KEY = "rt/neuron/state";

// 硬件配置
#define NUM_SERVOS 4
const int SERVO_PINS[NUM_SERVOS] = {12, 13, 14, 15}; // GPIO 引脚

// ==================== 全局变量 ====================

Servo servos[NUM_SERVOS];
z_owned_session_t session;
z_owned_publisher_t state_pub;
z_owned_subscriber_t cmd_sub;

// 当前关节位置 (度数)
float joint_positions[NUM_SERVOS] = {90, 90, 90, 90};

// ==================== Zenoh 回调 ====================

void cmd_callback(const z_sample_t *sample, void *arg) {
  // 解析命令 (简单 JSON: {"joint_0": 45, "joint_1": 90, ...})
  String payload = String((char *)sample->payload.start, sample->payload.len);

  Serial.print("📥 收到命令: ");
  Serial.println(payload);

  // 简单解析 (生产环境应使用 ArduinoJson)
  for (int i = 0; i < NUM_SERVOS; i++) {
    String key = "\"joint_" + String(i) + "\":";
    int idx = payload.indexOf(key);
    if (idx != -1) {
      int start = idx + key.length();
      int end = payload.indexOf(',', start);
      if (end == -1)
        end = payload.indexOf('}', start);

      float angle = payload.substring(start, end).toFloat();
      joint_positions[i] = constrain(angle, 0, 180);
      servos[i].write((int)joint_positions[i]);
    }
  }
}

// ==================== 初始化 ====================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n🤖 AGI-Walker ESP32 Neuron 启动中...");

  // 1. 初始化舵机
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(90); // 中位
  }
  Serial.println("✅ 舵机初始化完成");

  // 2. 连接 WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("📶 连接 WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi 已连接: " + WiFi.localIP().toString());

  // 3. 初始化 Zenoh
  z_owned_config_t config = z_config_default();
  zp_config_insert(z_loan(config), Z_CONFIG_MODE_KEY, z_string_make("client"));
  zp_config_insert(z_loan(config), Z_CONFIG_CONNECT_KEY,
                   z_string_make(ZENOH_ROUTER));

  session = z_open(z_move(config));
  if (!z_check(session)) {
    Serial.println("❌ Zenoh 会话创建失败!");
    while (1)
      delay(1000);
  }
  Serial.println("✅ Zenoh 会话已建立");

  // 4. 声明发布者
  state_pub =
      z_declare_publisher(z_loan(session), z_keyexpr(ZENOH_STATE_KEY), NULL);
  Serial.println("✅ 状态发布者已创建");

  // 5. 声明订阅者
  z_owned_closure_sample_t callback = z_closure(cmd_callback, NULL, NULL);
  cmd_sub = z_declare_subscriber(z_loan(session), z_keyexpr(ZENOH_CMD_KEY),
                                 z_move(callback), NULL);
  Serial.println("✅ 命令订阅者已创建");

  Serial.println("\n🚀 Neuron 就绪!");
}

// ==================== 主循环 ====================

void loop() {
  // 1. 处理 Zenoh 消息
  z_sleep_ms(10);

  // 2. 发布状态 (每 100ms)
  static unsigned long last_pub = 0;
  if (millis() - last_pub > 100) {
    last_pub = millis();

    // 构建状态 JSON
    String state = "{\"joint_positions\":[";
    for (int i = 0; i < NUM_SERVOS; i++) {
      state += String(joint_positions[i], 2);
      if (i < NUM_SERVOS - 1)
        state += ",";
    }
    state += "],\"timestamp\":" + String(millis()) + "}";

    // 发布
    z_publisher_put(z_loan(state_pub), (const uint8_t *)state.c_str(),
                    state.length(), NULL);
  }

  // 3. 读取传感器 (TODO: 添加 IMU/编码器)

  // 4. 本地安全检查
  // 例如: 检测过热、过流等
}
