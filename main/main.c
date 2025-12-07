#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_rom_sys.h"  // Required for esp_rom_delay_us
#include "driver/i2c.h"   // For I2C LCD
#include "driver/ledc.h"  // For PWM fan control
// WiFi credentials
#define WIFI_SSID "Mohanad"
#define WIFI_PASS "13572468"

// GPIO Pin definitions
#define DHT_PIN         GPIO_NUM_4
#define LDR_PIN         GPIO_NUM_34
#define PIR_PIN         GPIO_NUM_25
#define BUZZER_PIN      GPIO_NUM_23
#define LED_PIN         GPIO_NUM_12
#define FAN_PIN         GPIO_NUM_26  // PWM for fan speed control
#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22

// I2C LCD Configuration
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000
#define LCD_ADDR               0x27  // I2C address detected by scanner

// ADC channels
#define LDR_CHANNEL     ADC1_CHANNEL_6  // GPIO 34

// Fan PWM Configuration
#define FAN_PWM_TIMER          LEDC_TIMER_0
#define FAN_PWM_MODE           LEDC_LOW_SPEED_MODE
#define FAN_PWM_CHANNEL        LEDC_CHANNEL_0
#define FAN_PWM_DUTY_RES       LEDC_TIMER_8_BIT  // 8-bit resolution (0-255)
#define FAN_PWM_FREQUENCY      25000             // 25kHz for smooth fan operation

static const char *TAG = "ESP32_DASHBOARD";

// Global sensor data
static float humidity = 0;
static float temperature = 0;
static float temperatureF = 0;
static int ldrValue = 0;
static float lightPercentage = 0;
static bool motionDetected = false;
static int motionCount = 0;
static int noMotionSeconds = 0;
static bool buzzerOn = false;
static int buzzerDuration = 10;
static int lowLightSeconds = 0;
static bool ledOn = false;
static uint32_t sessionSeconds = 0;  // Session time in seconds
static bool sessionActive = true;     // Session is active when user is present

// Historical data storage (circular buffer for last 60 readings)
#define HISTORY_SIZE 60
static float temp_history[HISTORY_SIZE] = {0};
static float humid_history[HISTORY_SIZE] = {0};
static float light_history[HISTORY_SIZE] = {0};
static int motion_history[HISTORY_SIZE] = {0};
static int history_index = 0;
static int history_count = 0;
static uint8_t fanSpeed = 0;  // Track fan PWM duty (0-255)

// ADC calibration
static esp_adc_cal_characteristics_t adc_chars;

// WiFi event group
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
static int wifi_retry_count = 0;
#define MAX_WIFI_RETRY 5

// LCD I2C Commands
#define LCD_BACKLIGHT   0x08
#define LCD_NOBACKLIGHT 0x00
#define LCD_ENABLE      0x04

// I2C LCD Functions
static esp_err_t lcd_send_byte(uint8_t data, uint8_t mode) {
    uint8_t high = data & 0xF0;
    uint8_t low = (data << 4) & 0xF0;
    uint8_t buf[4];
    buf[0] = high | mode | LCD_BACKLIGHT | LCD_ENABLE;
    buf[1] = high | mode | LCD_BACKLIGHT;
    buf[2] = low | mode | LCD_BACKLIGHT | LCD_ENABLE;
    buf[3] = low | mode | LCD_BACKLIGHT;
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, buf, 4, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static void i2c_scanner(void) {
    ESP_LOGI(TAG, "I2C Scanner - Scanning bus...");
    uint8_t devices_found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t data = 0;
        esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, addr, &data, 1, pdMS_TO_TICKS(50));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at address 0x%02X", addr);
            devices_found++;
        }
    }
    if (devices_found == 0) {
        ESP_LOGE(TAG, "No I2C devices found! Check your connections!");
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s)", devices_found);
    }
}

static void lcd_init(void) {
    ESP_LOGI(TAG, "Initializing LCD at address 0x%02X...", LCD_ADDR);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Initialize in 8-bit mode first
    lcd_send_byte(0x33, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_byte(0x32, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Now in 4-bit mode
    lcd_send_byte(0x28, 0);  // Function set: 4-bit, 2 lines, 5x8 font
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_byte(0x0C, 0);  // Display ON, cursor OFF, blink OFF
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_byte(0x06, 0);  // Entry mode: increment cursor, no shift
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_byte(0x01, 0);  // Clear display
    vTaskDelay(pdMS_TO_TICKS(2));
    
    ESP_LOGI(TAG, "LCD initialization complete");
}

static void lcd_clear(void) {
    lcd_send_byte(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_byte(addr, 0);
}

// Print exactly 16 characters to LCD, padding with spaces if needed
static void lcd_print_line(const char* text) {
    char line[17];  // 16 chars + null
    int i;
    
    // Copy text or pad with spaces
    for (i = 0; i < 16; i++) {
        if (text[i] != '\0') {
            line[i] = text[i];
        } else {
            line[i] = ' ';  // Pad remaining with spaces
        }
    }
    line[16] = '\0';  // Null terminate
    
    // Send exactly 16 characters
    for (i = 0; i < 16; i++) {
        lcd_send_byte(line[i], 0x01);
    }
}

static void lcd_update_session_time(uint32_t seconds) {
    uint32_t hours = seconds / 3600;
    uint32_t minutes = (seconds % 3600) / 60;
    uint32_t secs = seconds % 60;
    
    char time_str[32];  // Larger buffer to avoid compiler warnings
    snprintf(time_str, sizeof(time_str), "    %02lu:%02lu:%02lu    ", hours, minutes, secs);
    
    lcd_set_cursor(0, 1);
    lcd_print_line(time_str);
}

// Fan PWM Functions
static void fan_init(void) {
    // Configure PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = FAN_PWM_MODE,
        .timer_num        = FAN_PWM_TIMER,
        .duty_resolution  = FAN_PWM_DUTY_RES,
        .freq_hz          = FAN_PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // Configure PWM channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = FAN_PWM_MODE,
        .channel        = FAN_PWM_CHANNEL,
        .timer_sel      = FAN_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = FAN_PIN,
        .duty           = 0,  // Start with fan off
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    ESP_LOGI(TAG, "Fan PWM initialized on GPIO %d", FAN_PIN);
}

static void fan_set_speed(float temp_celsius) {
    // Calculate fan speed based on temperature
    // Temperature ranges:
    // < 20°C: Fan off (0%)
    // 20-25°C: Fan low (30%)
    // 25-30°C: Fan medium (60%)
    // 30-35°C: Fan high (85%)
    // > 35°C: Fan max (100%)
    
    uint32_t duty = 0;
    
    if (temp_celsius < 20.0) {
        duty = 0;  // Fan off
    } else if (temp_celsius < 25.0) {
        // Linear interpolation: 20°C=30%, 25°C=60%
        duty = (uint32_t)(76 + (temp_celsius - 20.0) * 15.3);  // 30%-60%
    } else if (temp_celsius < 30.0) {
        // Linear interpolation: 25°C=60%, 30°C=85%
        duty = (uint32_t)(153 + (temp_celsius - 25.0) * 12.75);  // 60%-85%
    } else if (temp_celsius < 35.0) {
        // Linear interpolation: 30°C=85%, 35°C=100%
        duty = (uint32_t)(217 + (temp_celsius - 30.0) * 7.6);  // 85%-100%
    } else {
        duty = 255;  // Fan max
    }
    
    // Set PWM duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(FAN_PWM_MODE, FAN_PWM_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(FAN_PWM_MODE, FAN_PWM_CHANNEL));
    
    // Store fan speed for dashboard
    fanSpeed = (uint8_t)duty;
    
    ESP_LOGI(TAG, "Fan speed set to %lu/255 (%.1f%%) for temp %.1f°C", 
             duty, (duty * 100.0 / 255.0), temp_celsius);
}

// WiFi event group

// HTML dashboard (truncated for size - same as before)
static const char* html_page_part1 = 
"<!DOCTYPE html><html><head><meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
"<title>ESP32 Sensor Dashboard</title>"
"<script src=\"https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js\"></script>"
"<style>"
"*{margin:0;padding:0;box-sizing:border-box;}"
"body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;"
"background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);"
"min-height:100vh;padding:20px;}"
".container{max-width:1400px;margin:0 auto;}"
"h1{text-align:center;color:white;margin-bottom:30px;font-size:2.5em;"
"text-shadow:2px 2px 4px rgba(0,0,0,0.3);}"
".dashboard{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));"
"gap:20px;margin-bottom:30px;}"
".card{background:white;border-radius:15px;padding:20px;"
"box-shadow:0 10px 30px rgba(0,0,0,0.3);transition:transform 0.3s ease;}"
".card:hover{transform:translateY(-5px);}"
".card-header{display:flex;align-items:center;margin-bottom:10px;}"
".icon{font-size:2em;margin-right:10px;}"
".card-title{font-size:1em;color:#555;font-weight:600;}"
".card-value{font-size:2em;font-weight:bold;color:#333;margin:10px 0;}"
".card-unit{font-size:0.5em;color:#888;font-weight:normal;}"
".status-badge{display:inline-block;padding:5px 15px;border-radius:20px;"
"font-size:0.9em;font-weight:600;margin-top:5px;}"
".status-on{background:#4ade80;color:white;}"
".status-off{background:#94a3b8;color:white;}"
".charts{display:grid;grid-template-columns:repeat(auto-fit,minmax(400px,1fr));"
"gap:20px;margin-top:20px;}"
".chart-container{background:white;border-radius:15px;padding:20px;"
"box-shadow:0 10px 30px rgba(0,0,0,0.3);}"
".chart-title{font-size:1.2em;color:#333;font-weight:600;margin-bottom:15px;"
"text-align:center;}"
".status{text-align:center;color:white;font-size:0.9em;margin-top:20px;}"
"canvas{max-height:300px;}"
"</style></head><body><div class=\"container\">"
"<h1>🌡 ESP32 Sensor Dashboard</h1><div class=\"dashboard\">";

static const char* html_page_part2 = 
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">🌡</div>"
"<div class=\"card-title\">Temperature</div></div>"
"<div class=\"card-value\"><span id=\"temp\">--</span><span class=\"card-unit\">°C</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">💧</div>"
"<div class=\"card-title\">Humidity</div></div>"
"<div class=\"card-value\"><span id=\"humid\">--</span><span class=\"card-unit\">%</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">💡</div>"
"<div class=\"card-title\">Light Level</div></div>"
"<div class=\"card-value\"><span id=\"light\">--</span><span class=\"card-unit\">%</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">🚶</div>"
"<div class=\"card-title\">Motion</div></div>"
"<div class=\"card-value\"><span id=\"motion\">--</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">💡</div>"
"<div class=\"card-title\">LED Status</div></div>"
"<span id=\"led\" class=\"status-badge status-off\">OFF</span></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">🔔</div>"
"<div class=\"card-title\">Buzzer</div></div>"
"<span id=\"buzzer\" class=\"status-badge status-off\">OFF</span></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">🌀</div>"
"<div class=\"card-title\">Fan Speed</div></div>"
"<div class=\"card-value\"><span id=\"fan\">--</span><span class=\"card-unit\">%</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">⏱</div>"
"<div class=\"card-title\">Session Time</div></div>"
"<div class=\"card-value\"><span id=\"session\">00:00:00</span></div></div>"
"</div><div class=\"charts\">"
"<div class=\"chart-container\"><div class=\"chart-title\">📈 Temperature History</div>"
"<canvas id=\"tempChart\"></canvas></div>"
"<div class=\"chart-container\"><div class=\"chart-title\">💧 Humidity History</div>"
"<canvas id=\"humidChart\"></canvas></div>"
"<div class=\"chart-container\"><div class=\"chart-title\">💡 Light Level History</div>"
"<canvas id=\"lightChart\"></canvas></div>"
"<div class=\"chart-container\"><div class=\"chart-title\">🚶 Motion Events</div>"
"<canvas id=\"motionChart\"></canvas></div>"
"</div><div class=\"status\"><div id=\"status\">● Connected</div></div></div>"
"<script>"
"let tempChart,humidChart,lightChart,motionChart;"
"function initCharts(){"
"const commonOpts={responsive:true,maintainAspectRatio:true,"
"scales:{y:{beginAtZero:true},x:{display:false}},"
"plugins:{legend:{display:false}}};"
"tempChart=new Chart(document.getElementById('tempChart'),{"
"type:'line',data:{labels:[],datasets:[{label:'Temp (°C)',"
"data:[],borderColor:'#ef4444',backgroundColor:'rgba(239,68,68,0.1)',"
"tension:0.4,fill:true}]},options:commonOpts});"
"humidChart=new Chart(document.getElementById('humidChart'),{"
"type:'line',data:{labels:[],datasets:[{label:'Humidity (%)',"
"data:[],borderColor:'#3b82f6',backgroundColor:'rgba(59,130,246,0.1)',"
"tension:0.4,fill:true}]},options:commonOpts});"
"lightChart=new Chart(document.getElementById('lightChart'),{"
"type:'line',data:{labels:[],datasets:[{label:'Light (%)',"
"data:[],borderColor:'#fbbf24',backgroundColor:'rgba(251,191,36,0.1)',"
"tension:0.4,fill:true}]},options:commonOpts});"
"motionChart=new Chart(document.getElementById('motionChart'),{"
"type:'bar',data:{labels:[],datasets:[{label:'Motion Detected',"
"data:[],backgroundColor:'#10b981'}]},options:commonOpts});}"
"function updateCharts(data){"
"const labels=Array.from({length:data.historyCount},(_,i)=>i);"
"tempChart.data.labels=labels;tempChart.data.datasets[0].data=data.tempHistory;tempChart.update('none');"
"humidChart.data.labels=labels;humidChart.data.datasets[0].data=data.humidHistory;humidChart.update('none');"
"lightChart.data.labels=labels;lightChart.data.datasets[0].data=data.lightHistory;lightChart.update('none');"
"motionChart.data.labels=labels;motionChart.data.datasets[0].data=data.motionHistory;motionChart.update('none');}"
"function formatTime(secs){"
"const h=Math.floor(secs/3600);const m=Math.floor((secs%3600)/60);const s=secs%60;"
"return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(s).padStart(2,'0');}"
"function update(){fetch('/data').then(r=>r.json()).then(d=>{"
"document.getElementById('temp').textContent=d.temperature.toFixed(1);"
"document.getElementById('humid').textContent=d.humidity.toFixed(1);"
"document.getElementById('light').textContent=d.lightPercentage.toFixed(1);"
"document.getElementById('motion').textContent=d.motionDetected?'🔴 MOTION':'🟢 No Motion';"
"const ledEl=document.getElementById('led');"
"ledEl.textContent=d.ledOn?'ON':'OFF';"
"ledEl.className='status-badge '+(d.ledOn?'status-on':'status-off');"
"const buzzEl=document.getElementById('buzzer');"
"buzzEl.textContent=d.buzzerOn?'ALERT':'OFF';"
"buzzEl.className='status-badge '+(d.buzzerOn?'status-on':'status-off');"
"document.getElementById('fan').textContent=Math.round((d.fanSpeed/255)*100);"
"document.getElementById('session').textContent=formatTime(d.sessionSeconds);"
"updateCharts(d);"
"document.getElementById('status').textContent='● Connected';}).catch(e=>{"
"document.getElementById('status').textContent='● Error';});}initCharts();update();setInterval(update,2000);"
"</script></body></html>";

// DHT22 timing and reading functions (bit-banging)
#define DHT_TIMEOUT_US 1000

static inline void dht_delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

static bool dht22_read(gpio_num_t pin, float *temp, float *humid) {
    uint8_t data[5] = {0};
    
    // Send start signal
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    dht_delay_us(1000);
    gpio_set_level(pin, 1);
    dht_delay_us(30);
    
    // Wait for response
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    
    // Wait for low
    uint32_t timeout = 100;
    while (gpio_get_level(pin) == 1 && timeout--) dht_delay_us(1);
    if (timeout == 0) return false;
    
    // Wait for high
    timeout = 100;
    while (gpio_get_level(pin) == 0 && timeout--) dht_delay_us(1);
    if (timeout == 0) return false;
    
    // Wait for low
    timeout = 100;
    while (gpio_get_level(pin) == 1 && timeout--) dht_delay_us(1);
    if (timeout == 0) return false;
    
    // Read 40 bits
    for (int i = 0; i < 40; i++) {
        // Wait for high
        timeout = 100;
        while (gpio_get_level(pin) == 0 && timeout--) dht_delay_us(1);
        if (timeout == 0) return false;
        
        dht_delay_us(30);
        
        if (gpio_get_level(pin) == 1) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
        
        // Wait for low
        timeout = 100;
        while (gpio_get_level(pin) == 1 && timeout--) dht_delay_us(1);
    }
    
    // Verify checksum
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        return false;
    }
    
    // Convert to values
    *humid = ((data[0] << 8) | data[1]) / 10.0;
    *temp = (((data[2] & 0x7F) << 8) | data[3]) / 10.0;
    if (data[2] & 0x80) *temp = -*temp;
    
    return true;
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < MAX_WIFI_RETRY) {
            esp_wifi_connect();
            wifi_retry_count++;
            ESP_LOGI(TAG, "Disconnected, retry attempt %d/%d", wifi_retry_count, MAX_WIFI_RETRY);
        } else {
            ESP_LOGI(TAG, "Max retry reached, waiting 10s before retry...");
            vTaskDelay(pdMS_TO_TICKS(10000));
            wifi_retry_count = 0;
            esp_wifi_connect();
        }
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "WiFi Connected!");
        ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Dashboard URL: http://" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "========================================");
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize WiFi
static void wifi_init(void) {
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Set WiFi power save mode to NONE for stable connection
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialized, connecting to %s", WIFI_SSID);
}

// HTTP handlers
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_send_chunk(req, html_page_part1, strlen(html_page_part1));
    httpd_resp_send_chunk(req, html_page_part2, strlen(html_page_part2));
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req) {
    // Allocate JSON buffer on heap to avoid stack overflow
    char *json = (char*)malloc(4096);
    if (json == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Build JSON with current status and historical data
    int len = snprintf(json, 4096,
        "{\"temperature\":%.1f,\"temperatureF\":%.1f,\"humidity\":%.1f,"
        "\"ldrValue\":%d,\"lightPercentage\":%.1f,"
        "\"motionDetected\":%s,\"motionCount\":%d,"
        "\"ledOn\":%s,\"buzzerOn\":%s,\"fanSpeed\":%d,"
        "\"sessionActive\":%s,\"sessionSeconds\":%lu,"
        "\"tempHistory\":[",
        temperature, temperatureF, humidity, ldrValue, lightPercentage,
        motionDetected ? "true" : "false", motionCount,
        ledOn ? "true" : "false", buzzerOn ? "true" : "false", fanSpeed,
        sessionActive ? "true" : "false", sessionSeconds);
    
    // Add temperature history
    for (int i = 0; i < history_count; i++) {
        int idx = (history_index - history_count + i + HISTORY_SIZE) % HISTORY_SIZE;
        len += snprintf(json + len, 4096 - len, "%.1f%s", 
                       temp_history[idx], (i < history_count - 1) ? "," : "");
    }
    len += snprintf(json + len, 4096 - len, "],\"humidHistory\":[");
    
    // Add humidity history
    for (int i = 0; i < history_count; i++) {
        int idx = (history_index - history_count + i + HISTORY_SIZE) % HISTORY_SIZE;
        len += snprintf(json + len, 4096 - len, "%.1f%s", 
                       humid_history[idx], (i < history_count - 1) ? "," : "");
    }
    len += snprintf(json + len, 4096 - len, "],\"lightHistory\":[");
    
    // Add light history
    for (int i = 0; i < history_count; i++) {
        int idx = (history_index - history_count + i + HISTORY_SIZE) % HISTORY_SIZE;
        len += snprintf(json + len, 4096 - len, "%.1f%s", 
                       light_history[idx], (i < history_count - 1) ? "," : "");
    }
    len += snprintf(json + len, 4096 - len, "],\"motionHistory\":[");
    
    // Add motion history
    for (int i = 0; i < history_count; i++) {
        int idx = (history_index - history_count + i + HISTORY_SIZE) % HISTORY_SIZE;
        len += snprintf(json + len, 4096 - len, "%d%s", 
                       motion_history[idx], (i < history_count - 1) ? "," : "");
    }
    len += snprintf(json + len, 4096 - len, "],\"historyCount\":%d}", history_count);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    
    free(json);
    return ESP_OK;
}

// Start web server
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;  // Increase stack size from default 4096 to 8192
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);
        
        httpd_uri_t data = {
            .uri = "/data",
            .method = HTTP_GET,
            .handler = data_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &data);
        
        ESP_LOGI(TAG, "Web server started");
    }
    return server;
}

// Sensor reading task
static void sensor_task(void *pvParameters) {
    while (1) {
        // Read DHT22
        float temp, humid;
        if (dht22_read(DHT_PIN, &temp, &humid)) {
            temperature = temp;
            humidity = humid;
            temperatureF = temp * 1.8 + 32.0;
            
            // Control fan speed based on temperature
            fan_set_speed(temperature);
        }
        
        // Read LDR (inverted)
        ldrValue = adc1_get_raw(LDR_CHANNEL);
        lightPercentage = ((4095.0 - ldrValue) / 4095.0) * 100.0;
        
        // LED control based on light level
        if (lightPercentage < 50.0) {
            // Low light - increment counter
            lowLightSeconds += 1;  // Task runs every 1 second
            if (lowLightSeconds >= 5 && !ledOn) {
                gpio_set_level(LED_PIN, 1);
                ledOn = true;
                ESP_LOGI(TAG, "LED ON - Low light for 5+ seconds");
            }
        } else {
            // Good light - turn off LED and reset counter
            lowLightSeconds = 0;
            if (ledOn) {
                gpio_set_level(LED_PIN, 0);
                ledOn = false;
                ESP_LOGI(TAG, "LED OFF - Light level above 50%%");
            }
        }
        
        // Read PIR
        int pir = gpio_get_level(PIR_PIN);
        if (pir == 1 && !motionDetected) {
            motionCount++;
        }
        motionDetected = (pir == 1);
        
        // Buzzer control logic
        if (motionDetected) {
            // Motion detected - reset no-motion counter
            noMotionSeconds = 0;
            if (buzzerOn) {
                // Turn off buzzer and restart session from 0
                gpio_set_level(BUZZER_PIN, 0);
                buzzerOn = false;
                sessionActive = true;
                sessionSeconds = 0;  // Reset session time on motion after buzzer
                
                // Clear display completely
                lcd_clear();
                vTaskDelay(pdMS_TO_TICKS(50));  // Wait for clear to complete
                
                // Write line 1
                lcd_set_cursor(0, 0);
                lcd_print_line("Session Time:");
                vTaskDelay(pdMS_TO_TICKS(10));
                
                // Write line 2
                lcd_set_cursor(0, 1);
                lcd_print_line("    00:00:00");
                
                ESP_LOGI(TAG, "Buzzer OFF - Motion detected, session restarted");
            }
        } else {
            // No motion - increment counter
            noMotionSeconds += 1;  // Task runs every 1 second
            if (noMotionSeconds >= buzzerDuration && !buzzerOn) {
                // Time limit reached - trigger buzzer and reset session
                gpio_set_level(BUZZER_PIN, 1);
                buzzerOn = true;
                sessionActive = false;
                sessionSeconds = 0;  // Reset session time
                
                // Clear display completely
                lcd_clear();
                vTaskDelay(pdMS_TO_TICKS(50));  // Wait for clear to complete
                
                // Write line 1
                lcd_set_cursor(0, 0);
                lcd_print_line("User Away!");
                vTaskDelay(pdMS_TO_TICKS(10));
                
                // Write line 2
                lcd_set_cursor(0, 1);
                lcd_print_line("Session Reset");
                
                ESP_LOGI(TAG, "Buzzer ON - No motion for %d seconds, session reset", buzzerDuration);
            }
        }
        
        // Always increment session time when session is active (regardless of motion)
        // Only update LCD if buzzer is not on
        if (sessionActive && !buzzerOn) {
            sessionSeconds += 1;  // Add 1 second per cycle
            lcd_update_session_time(sessionSeconds);
        }
        
        // Store historical data (circular buffer)
        temp_history[history_index] = temperature;
        humid_history[history_index] = humidity;
        light_history[history_index] = lightPercentage;
        motion_history[history_index] = motionDetected ? 1 : 0;
        
        history_index = (history_index + 1) % HISTORY_SIZE;
        if (history_count < HISTORY_SIZE) {
            history_count++;
        }
        
        ESP_LOGI(TAG, "Temp: %.1f°C, Humid: %.1f%%, Light: %.1f%%, Motion: %s",
                 temperature, humidity, lightPercentage,
                 motionDetected ? "YES" : "NO");
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Run every 1 second
    }
}

void app_main(void) { 
    ESP_LOGI(TAG, "ESP32 Dashboard Starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Initialize I2C for LCD
    ESP_LOGI(TAG, "Initializing I2C: SDA=%d, SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C driver installed successfully");
    
    // Scan I2C bus
    i2c_scanner();
    
    // Initialize LCD
    lcd_init();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print_line("Session Time:");
    lcd_set_cursor(0, 1);
    lcd_print_line("    00:00:00");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Initialize Fan PWM
    fan_init();
    
    // Initialize Buzzer GPIO
    gpio_config_t buzzer_conf = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&buzzer_conf);
    gpio_set_level(BUZZER_PIN, 0);  // Start with buzzer OFF
    
    // Initialize LED GPIO
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_PIN, 0);  // Start with LED OFF
    
    // Initialize ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_CHANNEL, ADC_ATTEN_DB_12);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    
    // Initialize WiFi
    wifi_init();
    
    // Wait for WiFi connection
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    
    // Start web server
    start_webserver();
    
    // Create sensor reading task
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "System ready!");
}
