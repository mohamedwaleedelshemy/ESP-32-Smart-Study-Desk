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
// WiFi credentials
#define WIFI_SSID "Mohanad"
#define WIFI_PASS "13572468"

// GPIO Pin definitions
#define DHT_PIN         GPIO_NUM_4
#define LDR_PIN         GPIO_NUM_34
#define MIC_PIN         GPIO_NUM_35
#define PIR_PIN         GPIO_NUM_21

// ADC channels
#define LDR_CHANNEL     ADC1_CHANNEL_6  // GPIO 34
#define MIC_CHANNEL     ADC1_CHANNEL_7  // GPIO 35

static const char *TAG = "ESP32_DASHBOARD";

// Global sensor data
static float humidity = 0;
static float temperature = 0;
static float temperatureF = 0;
static int ldrValue = 0;
static float lightPercentage = 0;
static int micValue = 0;
static float soundPercentage = 0;
static bool motionDetected = false;
static int motionCount = 0;

// ADC calibration
static esp_adc_cal_characteristics_t adc_chars;

// WiFi event group
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// HTML dashboard (truncated for size - same as before)
static const char* html_page_part1 = 
"<!DOCTYPE html><html><head><meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
"<title>ESP32 Sensor Dashboard</title><style>"
"*{margin:0;padding:0;box-sizing:border-box;}"
"body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;"
"background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);"
"min-height:100vh;padding:20px;}"
".container{max-width:1200px;margin:0 auto;}"
"h1{text-align:center;color:white;margin-bottom:30px;font-size:2.5em;"
"text-shadow:2px 2px 4px rgba(0,0,0,0.3);}"
".dashboard{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));"
"gap:20px;margin-bottom:30px;}"
".card{background:white;border-radius:15px;padding:25px;"
"box-shadow:0 10px 30px rgba(0,0,0,0.3);transition:transform 0.3s ease;}"
".card:hover{transform:translateY(-5px);}"
".card-header{display:flex;align-items:center;margin-bottom:15px;}"
".icon{font-size:2.5em;margin-right:15px;}"
".card-title{font-size:1.2em;color:#555;font-weight:600;}"
".card-value{font-size:2.5em;font-weight:bold;color:#333;margin:15px 0;}"
".card-unit{font-size:0.5em;color:#888;font-weight:normal;}"
".status{text-align:center;color:white;font-size:0.9em;margin-top:20px;}"
"</style></head><body><div class=\"container\">"
"<h1>🌡️ ESP32 Sensor Dashboard</h1><div class=\"dashboard\">";

static const char* html_page_part2 = 
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">🌡️</div>"
"<div class=\"card-title\">Temperature</div></div>"
"<div class=\"card-value\"><span id=\"temp\">--</span><span class=\"card-unit\">°C</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">💧</div>"
"<div class=\"card-title\">Humidity</div></div>"
"<div class=\"card-value\"><span id=\"humid\">--</span><span class=\"card-unit\">%</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">💡</div>"
"<div class=\"card-title\">Light</div></div>"
"<div class=\"card-value\"><span id=\"light\">--</span><span class=\"card-unit\">%</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">🎤</div>"
"<div class=\"card-title\">Sound</div></div>"
"<div class=\"card-value\"><span id=\"sound\">--</span><span class=\"card-unit\">%</span></div></div>"
"<div class=\"card\"><div class=\"card-header\"><div class=\"icon\">🚶</div>"
"<div class=\"card-title\">Motion</div></div>"
"<div class=\"card-value\"><span id=\"motion\">--</span></div></div>"
"</div><div class=\"status\"><div id=\"status\">● Connected</div></div></div>"
"<script>function update(){fetch('/data').then(r=>r.json()).then(d=>{"
"document.getElementById('temp').textContent=d.temperature.toFixed(1);"
"document.getElementById('humid').textContent=d.humidity.toFixed(1);"
"document.getElementById('light').textContent=d.lightPercentage.toFixed(1);"
"document.getElementById('sound').textContent=d.soundPercentage.toFixed(1);"
"document.getElementById('motion').textContent=d.motionDetected?'🔴 MOTION':'🟢 No Motion';"
"document.getElementById('status').textContent='● Connected';}).catch(e=>{"
"document.getElementById('status').textContent='● Error';});}update();setInterval(update,2000);"
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
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
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
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialized");
}

// HTTP handlers
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_send_chunk(req, html_page_part1, strlen(html_page_part1));
    httpd_resp_send_chunk(req, html_page_part2, strlen(html_page_part2));
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req) {
    char json[256];
    snprintf(json, sizeof(json),
        "{\"temperature\":%.1f,\"temperatureF\":%.1f,\"humidity\":%.1f,"
        "\"ldrValue\":%d,\"lightPercentage\":%.1f,\"micValue\":%d,"
        "\"soundPercentage\":%.1f,\"motionDetected\":%s,\"motionCount\":%d}",
        temperature, temperatureF, humidity, ldrValue, lightPercentage,
        micValue, soundPercentage, motionDetected ? "true" : "false", motionCount);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// Start web server
static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
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
        }
        
        // Read LDR (inverted)
        ldrValue = adc1_get_raw(LDR_CHANNEL);
        lightPercentage = ((4095.0 - ldrValue) / 4095.0) * 100.0;
        
        // Read Microphone
        micValue = adc1_get_raw(MIC_CHANNEL);
        soundPercentage = (micValue / 4095.0) * 100.0;
        
        // Read PIR
        int pir = gpio_get_level(PIR_PIN);
        if (pir == 1 && !motionDetected) {
            motionCount++;
        }
        motionDetected = (pir == 1);
        
        ESP_LOGI(TAG, "Temp: %.1f°C, Humid: %.1f%%, Light: %.1f%%, Sound: %.1f%%, Motion: %s",
                 temperature, humidity, lightPercentage, soundPercentage,
                 motionDetected ? "YES" : "NO");
        
        vTaskDelay(pdMS_TO_TICKS(2000));
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
    
    // Initialize ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(MIC_CHANNEL, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    
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