# ESP-IDF 零基础学习指南

**for OpenNeuro Project**

**目标**: 在ESP32硬件到货前，掌握ESP-IDF开发的核心技能

**预计学习时间**: 3-5天  
**难度**: 初级到中级

---

## 📚 学习路径

### Day 1: 环境搭建和Hello World
- 安装ESP-IDF
- 理解项目结构
- 编译第一个程序

### Day 2: FreeRTOS基础
- 任务管理
- 队列和信号量
- 定时器

### Day 3: WiFi和网络
- WiFi station模式
- TCP/UDP通信
- HTTP客户端

### Day 4: 外设驱动
- GPIO控制
- I2C传感器
- PWM输出

### Day 5: 实战项目
- 综合示例
- 调试技巧
- 性能优化

---

## 🚀 Day 1: 环境搭建和Hello World

### 1.1 安装ESP-IDF

#### Windows (推荐使用WSL)

**方法1: 使用WSL (推荐)**
```bash
# 在WSL Ubuntu中
sudo apt update
sudo apt install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# 克隆ESP-IDF
mkdir -p ~/esp
cd ~/esp
git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git

# 安装工具链
cd esp-idf
./install.sh esp32,esp32s3

# 设置环境变量
. ./export.sh
```

**方法2: Windows原生工具**
- 下载ESP-IDF Tools Installer
- https://dl.espressif.com/dl/esp-idf/

#### Linux/macOS

```bash
# 安装依赖
# Ubuntu/Debian
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# macOS
brew install cmake ninja dfu-util

# 克隆ESP-IDF
mkdir -p ~/esp
cd ~/esp
git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git

# 安装
cd ~/esp/esp-idf
./install.sh esp32,esp32s3

# 环境变量（每次新终端都要执行）
. ~/esp/esp-idf/export.sh

# 或添加到.bashrc/.zshrc
echo 'alias get_idf=". ~/esp/esp-idf/export.sh"' >> ~/.bashrc
```

### 1.2 Hello World项目

#### 创建项目

```bash
# 设置环境
. ~/esp/esp-idf/export.sh

# 复制示例
cd ~/esp
cp -r esp-idf/examples/get-started/hello_world my_hello_world
cd my_hello_world

# 查看项目结构
tree -L 2
```

**项目结构**:
```
my_hello_world/
├── CMakeLists.txt          # 项目配置
├── main/
│   ├── CMakeLists.txt      # 组件配置
│   └── hello_world_main.c  # 主程序
└── README.md
```

#### 理解代码

**文件**: `main/hello_world_main.c`

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

static const char *TAG = "hellowor ld";

void app_main(void)
{
    printf("Hello world!\n");
    
    // 芯片信息
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    
    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);
    
    ESP_LOGI(TAG, "%dMB %s flash\n", 
            spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    
    // 重启倒计时
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延时1秒
    }
    
    ESP_LOGI(TAG, "Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
```

**关键概念**:
1. `app_main()` - 入口函数
2. `ESP_LOGI()` - 日志输出
3. `vTaskDelay()` - FreeRTOS延时

#### 编译和运行

```bash
# 设置目标芯片
idf.py set-target esp32s3  # 或 esp32

# 配置项目（可选）
idf.py menuconfig

# 编译
idf.py build

# 连接ESP32后烧录
idf.py -p /dev/ttyUSB0 flash  # Linux
# 或
idf.py -p COM3 flash          # Windows

# 查看输出
idf.py -p /dev/ttyUSB0 monitor

# 或一条命令完成
idf.py -p /dev/ttyUSB0 flash monitor
```

**预期输出**:
```
Hello world!
I (320) helloworld: This is esp32s3 chip with 2 CPU core(s), WiFi/BLE, 
I (320) helloworld: silicon revision 0, 
I (330) helloworld: 8MB external flash
Restarting in 10 seconds...
Restarting in 9 seconds...
...
```

---

## 🔧 Day 2: FreeRTOS基础

### 2.1 任务（Tasks）

ESP-IDF基于FreeRTOS，理解任务是关键。

#### 创建任务示例

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "tasks_demo";

// 任务1: 每秒打印一次
void task1(void *pvParameter)
{
    int count = 0;
    while(1) {
        ESP_LOGI(TAG, "Task 1: Count = %d", count++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1秒
    }
}

// 任务2: 每500ms打印一次
void task2(void *pvParameter)
{
    int count = 0;
    while(1) {
        ESP_LOGI(TAG, "Task 2: Count = %d", count++);
        vTaskDelay(500 / portTICK_PERIOD_MS);   // 500ms
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting FreeRTOS tasks demo");
    
    // 创建任务1
    xTaskCreate(
        task1,          // 任务函数
        "Task1",        // 任务名称
        2048,           // 栈大小（字节）
        NULL,           // 参数
        5,              // 优先级（数字越大优先级越高）
        NULL            // 任务句柄
    );
    
    // 创建任务2
    xTaskCreate(task2, "Task2", 2048, NULL, 5, NULL);
    
    // app_main结束后，任务继续运行
}
```

**关键API**:
- `xTaskCreate()` - 创建任务
- `vTaskDelay()` - 延时
- `vTaskDelete()` - 删除任务

### 2.2 队列（Queues）

任务间通信的主要方式。

```c
#include "freertos/queue.h"

QueueHandle_t xQueue;

void sender_task(void *pvParameter)
{
    int counter = 0;
    while(1) {
        // 发送数据到队列
        if(xQueueSend(xQueue, &counter, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "Sent: %d", counter);
            counter++;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void receiver_task(void *pvParameter)
{
    int received;
    while(1) {
        // 从队列接收
        if(xQueueReceive(xQueue, &received, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "Received: %d", received);
        }
    }
}

void app_main(void)
{
    // 创建队列（容量10，每个元素int大小）
    xQueue = xQueueCreate(10, sizeof(int));
    
    xTaskCreate(sender_task, "Sender", 2048, NULL, 5, NULL);
    xTaskCreate(receiver_task, "Receiver", 2048, NULL, 5, NULL);
}
```

---

## 📡 Day 3: WiFi和网络

### 3.1 WiFi Station模式

连接到现有WiFi网络。

```c
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#define WIFI_SSID      "YourSSID"
#define WIFI_PASS      "YourPassword"

static const char *TAG = "wifi_station";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retry connecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void wifi_init_sta(void)
{
    // 初始化NVS（WiFi需要）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化TCP/IP栈
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // WiFi配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册事件处理器
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // WiFi配置
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi init finished.");
}

void app_main(void)
{
    wifi_init_sta();
}
```

### 3.2 使用menuconfig配置WiFi

更优雅的方式是通过menuconfig：

```bash
idf.py menuconfig
```

导航到：
```
Component config → WiFi
Example Configuration
```

然后在代码中：
```c
#define WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
```

---

## 🔌 Day 4: 外设驱动

### 4.1 GPIO - LED闪烁

```c
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_2  // 板载LED通常是GPIO2

void app_main(void)
{
    // 配置GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // 闪烁
    while(1) {
        gpio_set_level(LED_PIN, 1);  // 开
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);  // 关
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
```

### 4.2 PWM - 呼吸灯

```c
#include "driver/ledc.h"

void app_main(void)
{
    // LEDC配置
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = GPIO_NUM_2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
    };
    ledc_channel_config(&ledc_channel);
    
    // 呼吸效果
    while(1) {
        // 渐亮
        for(int duty = 0; duty <= 8192; duty += 100) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        // 渐暗
        for(int duty = 8192; duty >= 0; duty -= 100) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}
```

---

## 💡 Day 5: 实战项目

### 项目: WiFi控制LED

结合WiFi和GPIO，创建一个简单的IoT设备。

**功能**:
1. 连接WiFi
2. 创建HTTP服务器
3. 通过网页控制LED

**提示**: 使用ESP-IDF的`esp_http_server`组件。

---

## 🐛 调试技巧

### 1. 串口监视器

```bash
idf.py monitor

# 快捷键
Ctrl+] - 退出
Ctrl+T Ctrl+H - 帮助
Ctrl+T Ctrl+R - 重启芯片
```

### 2. 日志级别

```c
// 设置日志级别
esp_log_level_set("*", ESP_LOG_INFO);
esp_log_level_set("wifi", ESP_LOG_DEBUG);

// 使用不同级别
ESP_LOGE(TAG, "Error");    // 错误
ESP_LOGW(TAG, "Warning");  // 警告
ESP_LOGI(TAG, "Info");     // 信息
ESP_LOGD(TAG, "Debug");    // 调试
ESP_LOGV(TAG, "Verbose");  // 详细
```

### 3. 断言

```c
// 运行时检查
assert(ptr != NULL);

// ESP-IDF特定
ESP_ERROR_CHECK(esp_wifi_init(&cfg));  // 失败会重启
```

---

## 📚 学习资源

### 官方文档
- [ESP-IDF编程指南](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [API参考](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/)

### 示例代码
```bash
# ESP-IDF自带200+示例
cd ~/esp/esp-idf/examples
ls
```

重要示例目录:
- `get-started/` - 入门
- `wifi/` - WiFi相关
- `peripherals/` - 外设
- `bluetooth/` - 蓝牙
- `protocols/` - 网络协议

### 视频教程
- [Espressif官方YouTube](https://www.youtube.com/c/EspressifSystems)
- [ESP32教程 - Kolban](https://www.youtube.com/playlist?list=PLB-czhEQLJbcR4FoXc82pR9wkZuXzp5p1)

---

## ✅ 学习检查清单

### Day 1
- [ ] ESP-IDF成功安装
- [ ] Hello World编译运行
- [ ] 理解项目结构

### Day 2
- [ ] 创建多个FreeRTOS任务
- [ ] 使用队列通信
- [ ] 理解优先级和调度

### Day 3
- [ ] 连接WiFi成功
- [ ] 获取IP地址
- [ ] 理解事件系统

### Day 4
- [ ] 控制GPIO
- [ ] 使用PWM
- [ ] 读取传感器（可选）

### Day 5
- [ ] 完成综合项目
- [ ] 掌握调试技巧

---

## 🎯 为OpenNeuro准备

完成以上学习后，您应该能够：

1. ✅ 理解OpenNeuro ESP32固件代码
2. ✅ 修改WiFi配置
3. ✅ 调试Zenoh-Pico集成
4. ✅ 添加自定义传感器
5. ✅ 优化性能和功耗

---

## 🚀 下一步

学完ESP-IDF后：
1. 复习OpenNeuro的`main.c`代码
2. 理解Zenoh-Pico API
3. 准备硬件到货后立即开发

**预计学习时间**: 每天2-3小时，共15-20小时

**成功标准**: 能独立创建WiFi+LED控制项目
