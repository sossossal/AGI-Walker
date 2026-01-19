/*
 * OpenNeuro Zone Controller Firmware (MVP Reference)
 * Target: STM32H753
 *
 * 功能:
 * 1. Zenoh-Pico over Ethernet (LwIP)
 * 2. Motor Control Loop (1kHz)
 * 3. IMC-22 Interface (SPI)
 */

#include "main.h"
#include "lwip.h"
#include "zenoh-pico.h"

// --- Configuration ---
#define ZONE_ID 1
#define ZENOH_ROUTER_IP "192.168.1.100"
#define ZENOH_PORT 7447

// --- Zenoh Globals ---
z_owned_session_t session;
z_owned_publisher_t pub_state;
z_owned_subscriber_t sub_cmd;

// --- Motor State ---
typedef struct {
  float position;
  float velocity;
  float torque;
} motor_state_t;

motor_state_t motors[6];
float target_positions[6];

// --- Callbacks ---

// 接收控制命令
void on_cmd(const z_sample_t *sample, void *arg) {
  // TODO: Parse JSON/Protobuf payload
  // update target_positions[]
}

// --- Tasks ---

// 1kHz Motor Control Loop (High Priority)
void MotorControlTask(void *argument) {
  for (;;) {
    // Read Encoders (HAL_SPI_TransmitReceive...)

    // PID Calculation
    for (int i = 0; i < 6; i++) {
      // float output = PID_Update(target_positions[i], motors[i].position);
      // Set PWM (HAL_TIM_PWM_Start...)
    }

    osDelay(1); // 1ms
  }
}

// 100Hz Communication Loop (Medium Priority)
void CommTask(void *argument) {
  // Zenoh Init
  z_owned_config_t config = z_config_default();
  zp_config_insert(z_config_loan(&config), "mode", "client");
  zp_config_insert(z_config_loan(&config), "connect/endpoints",
                   "tcp/" ZENOH_ROUTER_IP ":7447");

  session = z_open(z_move(config));

  // Declare Resources
  z_keyexpr_t key_state = z_keyexpr("/zone/1/state");
  pub_state = z_declare_publisher(z_session_loan(&session), key_state, NULL);

  z_keyexpr_t key_cmd = z_keyexpr("/zone/1/cmd");
  sub_cmd = z_declare_subscriber(z_session_loan(&session), key_cmd,
                                 z_closure(on_cmd, NULL, NULL), NULL);

  char payload[256];

  for (;;) {
    // Prepare State Payload
    sprintf(payload, "{\"ts\":%lu, \"pos\":[%.2f, %.2f...]}", HAL_GetTick(),
            motors[0].position, motors[1].position);

    // Publish
    z_publisher_put(z_publisher_loan(&pub_state), (uint8_t *)payload,
                    strlen(payload), NULL);

    // Handle Zenoh I/O
    // z_session_check(&session); // In Pico, often handled internally or via
    // loop

    osDelay(10); // 10ms -> 100Hz
  }
}

// IMC-22 Inference Task (Event Driven)
void AIInferenceTask(void *argument) {
  for (;;) {
    // Wait for data ready signal
    // SPI Transfer to FPGA/IMC-22
    // Read Result
    osDelay(20);
  }
}

void main() {
  HAL_Init();
  SystemClock_Config();
  MX_LWIP_Init();

  // Create Tasks
  xTaskCreate(MotorControlTask, "Motor", 1024, NULL, osPriorityRealtime, NULL);
  xTaskCreate(CommTask, "Comm", 4096, NULL, osPriorityNormal, NULL);
  xTaskCreate(AIInferenceTask, "AI", 2048, NULL, osPriorityHigh, NULL);

  vTaskStartScheduler();
}
