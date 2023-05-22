// // Nicholas Hardy, Marybel Boujaoude, Hassan Hijazi, Riya Deokar
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include <math.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gptimer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   // Added in 2023..
#include <inttypes.h> // Added in 2023
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // Added in 2023
#include "esp_log.h"
#include "esp_system.h"    // Added in 2023
#include "driver/rmt_tx.h" // Modified in 2023
#include "soc/rmt_reg.h"   // Not needed?
#include "driver/uart.h"
// #include "driver/periph_ctrl.h"
#include "driver/gptimer.h"       // Added in 2023
#include "clk_tree.h"             // Added in 2023
#include "driver/gpio.h"          // Added in 2023
#include "driver/mcpwm_prelude.h" // Added in 2023
#include "driver/ledc.h"          // Added in 2023
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "driver/adc.h"
#include <math.h>
#include "esp_adc_cal.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#define SLAVE_ADDR 0x70              // alphanumeric address
#define OSC 0x21                     // oscillator cmd
#define HT16K33_BLINK_DISPLAYON 0x01 // Display on cmd
#define HT16K33_BLINK_OFF 0          // Blink off cmd
#define HT16K33_BLINK_CMD 0x80       // Blink cmd
#define HT16K33_CMD_BRIGHTNESS 0xE0  // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   // i2c master clock freq
#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define ACK_CHECK_EN true                   // i2c master will check ack
#define ACK_CHECK_DIS false                 // i2c master will not check ack
#define ACK_VAL 0x00                        // i2c ack value
#define NACK_VAL 0xFF                       // i2c nack value

static const char *TAG_1 = "example";

////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////     LIDAR      //////////////////////////////////
#define LIDARLite_ADDRESS 0x62 // Default I2C Address of LIDAR-Lite.
#define RegisterMeasure 0x00   // Register to write to initiate ranging.
#define MeasureValue 0x04      // Value to initiate ranging.
#define RegisterHighLowB 0x8f  // Register to get both High and Low bytes in 1 call.

#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 400000   // i2c master clock freq
#define ACK_CHECK_EN 0x1

#define Lidar1 15
#define Lidar2 32
#define Lidar3 14

#define Lidar1_addr 0x58
#define Lidar2_addr 0x60
#define Lidar3_addr 0x62

#define WIFI_SSID "Group_1"
#define WIFI_PASS "smartsys"
#define HOST_IP_ADDR "192.168.1.24"
#define PORT_SND 8081
char PAYLOAD[20];
// float PAYLOAD;

#define PORT_RCV 8081
#define BUFFER_SIZE 1024

static const char *TAG = "udp_client";

//*******************************************************//

// MCPWM defintions -- 2023: modified
#define MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define MCPWM_FREQ_HZ 38000                // 38KHz PWM -- 1/38kHz = 26.3us
#define MCPWM_FREQ_PERIOD 263              // 263 ticks = 263 * 0.1us = 26.3us
#define MCPWM_GPIO_NUM 25

// LEDC definitions -- 2023: modified
// NOT USED / altnernative to MCPWM above
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO 25
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES 6      // Set duty resolution to 6 bits
#define LEDC_DUTY 32         // Set duty to 50%. ((2^6) - 1) * 50% = 32
#define LEDC_FREQUENCY 38000 // Frequency in Hertz. 38kHz

// UART definitions -- 2023: no changes
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)
#define BUF_SIZE2 (32)

// Hardware interrupt definitions -- 2023: no changes
#define GPIO_INPUT_IO_1 4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_1

// LED Output pins definitions -- 2023: minor changes
#define BLUEPIN 14
#define GREENPIN 32
#define REDPIN 15
#define ONBOARD 13
#define GPIO_OUTPUT_PIN_SEL ((1ULL << BLUEPIN) | (1ULL << GREENPIN) | (1ULL << REDPIN) | (1ULL << ONBOARD))

// Default ID/color
#define ID 3
#define COLOR 'R'

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B; // START BYTE that UART looks for
// char FIB_in = (char) ID;
char myColor = (char)COLOR;
int len_out = 5;
char qr_code = '0';
char wasdfcontrols = '0';
uint32_t FIB_in = 0x10;
uint32_t KEY_in = 0x30;

// uint32_t FIB_out;
// uint32_t KEY_out;

#define FIB_out FIB_in
#define KEY_out KEY_in

// You can change the values of the output variables later if needed
// SID_out = 0x20;
// FIB_out = 0x30;
// KEY_out = 0x2020;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;               // 2023: no changes
static QueueHandle_t gpio_evt_queue = NULL; // 2023: Changed
// static xQueueHandle_t timer_queue; -- 2023: removed

// A simple structure to pass "events" to main task -- 2023: modified
typedef struct
{
    uint64_t event_count;
} example_queue_element_t;

// Create a FIFO queue for timer-based events -- Modified
example_queue_element_t ele;
QueueHandle_t timer_queue;

// System tags for diagnostics -- 2023: modified
// static const char *TAG_SYSTEM = "ec444: system";       // For debug logs
static const char *TAG_TIMER = "ec444: timer"; // For timer logs
static const char *TAG_UART = "ec444: uart";   // For UART logs

// Button interrupt handler -- add to queue -- 2023: no changes
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Timmer interrupt handler -- Callback timer function -- 2023: modified
// Note we enabled time for auto-reload
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue1 = (QueueHandle_t)user_data;
    // Retrieve count value and send to queue
    example_queue_element_t ele = {
        .event_count = edata->count_value};
    xQueueSendFromISR(timer_queue1, &ele, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}

// Utilities ///////////////////////////////////////////////////////////////////

// Checksum -- 2023: no changes
char genCheckSum(char *p, int len)
{
    char temp = 0;
    for (int i = 0; i < len; i++)
    {
        temp = temp ^ p[i];
    }
    // printf("%X\n",temp);  // Diagnostic

    return temp;
}
bool checkCheckSum(uint8_t *p, int len)
{
    char temp = (char)0;
    bool isValid;
    for (int i = 0; i < len - 1; i++)
    {
        temp = temp ^ p[i];
    }
    // printf("Check: %02X ", temp); // Diagnostic
    if (temp == p[len - 1])
    {
        isValid = true;
    }
    else
    {
        isValid = false;
    }
    return isValid;
}

// MCPWM Initialize -- 2023: this is to create 38kHz carrier
static void pwm_init()
{

    // Create timer
    mcpwm_timer_handle_t pwm_timer = NULL;
    mcpwm_timer_config_t pwm_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .period_ticks = MCPWM_FREQ_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&pwm_timer_config, &pwm_timer));

    // Create operator
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    // Connect timer and operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, pwm_timer));

    // Create comparator from the operator
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // Create generator from the operator
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = MCPWM_GPIO_NUM,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the duty cycle is 50%
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 132));
    // CANNOT FIGURE OUT HOW MANY TICKS TO COMPARE TO TO GET 50%

    // Set generator action on timer and compare event
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    // Enable and start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP));
}

// Configure UART -- 2023: minor changes
static void uart_init()
{
    // Basic configs
    const uart_config_t uart_config = {
        .baud_rate = 1200, // Slow BAUD rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};
    uart_param_config(UART_NUM_1, &uart_config);

    // Set UART pins using UART0 default pins
    uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Reverse receive logic line
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

    // Install UART driver
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs -- 2023: modified
static void led_init()
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
}

// Configure timer -- 2023: Modified
static void alarm_init()
{

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // Set alarm callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue));

    // Enable timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG_TIMER, "Start timer, update alarm value dynamically and auto reload");
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,                  // counter will reload with 0 on alarm event
        .alarm_count = 10 * 1000000,        // period = 10*1s = 10s
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

// Tasks

// LED task to light LED based on traffic state -- 2023: no changes
// void led_task()
// {
//     while (1)
//     {
//         switch ((int)myColor)
//         {
//         case 'R': // Red
//             gpio_set_level(GREENPIN, 0);
//             gpio_set_level(REDPIN, 1);
//             gpio_set_level(BLUEPIN, 0);
//             // printf("Current state: %c\n",status);
//             break;
//         case 'Y': // Yellow
//             gpio_set_level(GREENPIN, 0);
//             gpio_set_level(REDPIN, 0);
//             gpio_set_level(BLUEPIN, 1);
//             // printf("Current state: %c\n",status);
//             break;
//         case 'G': // Green
//             gpio_set_level(GREENPIN, 1);
//             gpio_set_level(REDPIN, 0);
//             gpio_set_level(BLUEPIN, 0);
//             // printf("Current state: %c\n",status);
//             break;
//         }
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }

// LED task to blink onboard LED based on ID -- 2023: no changes
// void id_task()
// {
//     while (1)
//     {
//         for (int i = 0; i < (int)FIB_in; i++)
//         {
//             gpio_set_level(ONBOARD, 1);
//             vTaskDelay(200 / portTICK_PERIOD_MS);
//             gpio_set_level(ONBOARD, 0);
//             vTaskDelay(200 / portTICK_PERIOD_MS);
//         }
//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

// Timer task -- R (10 seconds), G (10 seconds), Y (10 seconds) -- 2023: modified
static void timer_evt_task(void *arg)
{
    while (1)
    {
        // Transfer from queue and do something if triggered
        if (xQueueReceive(timer_queue, &ele, pdMS_TO_TICKS(2000)))
        {
            printf("Action!\n");
            if (myColor == 'R')
            {
                myColor = 'G';
            }
            else if (myColor == 'G')
            {
                myColor = 'Y';
            }
            else if (myColor == 'Y')
            {
                myColor = 'R';
            }
        }
    }
}

// UDP Client Code*****************************************//

void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "Connected to AP");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Disconnected from AP");
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "Got IP address");
    }
}

void udp_server_task(void *pvParameters)
{
    struct sockaddr_in addr;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT_RCV);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    int err = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT_RCV);
    while (1)
    {
        char rx_buffer[BUFFER_SIZE];
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0)
        {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        }
        else
        {
            rx_buffer[len] = '\0';
            ESP_LOGI(TAG, "Received %d bytes from %s:%d", len, inet_ntoa(source_addr.sin_addr), ntohs(source_addr.sin_port));
            printf("Received Message: %s\n", rx_buffer);
            qr_code = rx_buffer[0];
            wasdfcontrols = rx_buffer[0];
        }
    }
    vTaskDelete(NULL);
}

float distance1, distance2, distance3, prev_distance1;
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

static const uint16_t alphafonttable[] = {
    0b0000000000000000, //  space
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0100000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000000000, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111,

};

int alpha_oscillator()
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return ret;
}

// Set blink rate to off
int no_blink()
{
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val)
{
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return ret;
}

static void test_alpha_display(float sum)
{
    printf(">> Press Button to start timer & press again to restart: \n");

    int ret;
    ret = alpha_oscillator();
    ret = no_blink();
    ret = set_brightness_max(0xF);

    // Write to characters to buffer
    uint16_t displaybuffer[8];
    displaybuffer[0] = 0b0101001000000001; // #
    displaybuffer[1] = 0b0000000000000000; // blank
    displaybuffer[2] = 0b0100000000000000; // .
    displaybuffer[3] = 0b0000110000111111; // 0

    char nums[4];
    itoa(sum, nums, 10);
    // while (1) {
    if (sum < 10)
    {
        int val = nums[0];
        displaybuffer[0] = alphafonttable[val];
        displaybuffer[1] = 0b0000000000000000;
        displaybuffer[2] = 0b0100000000000000;
        displaybuffer[3] = 0b0000110000111111;
    }
    else if (sum >= 10 && sum < 100)
    {
        for (int i = 0; i < 2; i++)
        {
            int val = nums[i];
            displaybuffer[i] = alphafonttable[val];
        }
        displaybuffer[2] = 0b0100000000000000;
        displaybuffer[3] = 0b0000110000111111;
    }
    else if (sum >= 100 && sum < 1000)
    {
        for (int i = 0; i < 3; i++)
        {
            int val = nums[i];
            displaybuffer[i] = alphafonttable[val];
        }
        displaybuffer[3] = 0b0000110000111111;
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            int val = nums[i];
            displaybuffer[i] = alphafonttable[val];
        }
    }
    // Send commands characters to display over I2C
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i = 0; i < 8; i++)
    {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd4);
}

////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////    ESC/SERVO     //////////////////////////////////

// Please consult the datasheet of your servo before changing the following parameters
#define ESC_MIN_PULSEWIDTH_US 900      // Minimum pulse width in microsecond
#define ESC_MAX_PULSEWIDTH_US 2100     // Maximum pulse width in microsecond
#define ESC_NEUTRAL_PULSEWIDTH_US 1500 // neutral value for esc to calibrate

/// STEERING SERVO
#define S_SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define S_SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond

#define SERVO_MIN_DEGREE -90 // Minimum angle
#define SERVO_MAX_DEGREE 90  // Maximum angle

#define STEERING_SERVO_PULSE_GPIO 26 // GPIO connects to the PWM signal for servo 1
#define ESC_SERVO_PULSE_GPIO 25      // GPIO connects to the PWM signal for servo 2

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

#define MAX_DUTY_LENGTH 8191 // Maximum duty cycle of 8191

//////////// ULTRASONIC SENSOR /////////
////////////////////////////////////////////////////////////////////////////////////
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  // Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_3; // GPIO39 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
//////////// ULTRASONIC SENSOR /////////
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
///////////////////////////        LIDAR i2c       /////////////////////////////////
int testConnection(uint8_t devAddr, int32_t timeout)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}
// Utility function to scan for i2c device
static void i2c_scanner()
{
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."
           "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++)
    {
        if (testConnection(i, scanTimeout) == ESP_OK)
        {
            printf("- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
    {
        printf("- No I2C devices found!"
               "\n");
    }
}
// i2c init from Master init
static void i2c_master_init()
{
    printf("\n>> i2c Config\n"); // For debugging
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    // Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                        // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
    conf.clk_flags = 0;
    err = i2c_param_config(i2c_master_port, &conf); // Configure
    if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK)
    {
        printf("- initialized: yes\n");
    }

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Read and write to register
// Functions //////////

// Function to write one byte to register (single byte write)
void writeRegister(uint8_t reg, uint8_t data, int addr)
{

    // create i2c communication init
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                               // 1. Start (Master write start)
    i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, I2C_MASTER_ACK); // (Master write follower add + write bit)
    // wait for salve to ack
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); // (Master write register address)
    // wait for follower to ack
    i2c_master_write_byte(cmd, data, I2C_MASTER_ACK); // master write data
    // wait for follower to ack
    i2c_master_stop(cmd); // 11. Stop
    // i2c communication done and delete
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    // no return here since data is written by master onto follower
}

// Function to read register (single byte read)
uint16_t readRegister(uint8_t reg, int addr)
{

    uint8_t data1; // first byte MSB
    uint8_t data2; // second byte LSB

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();

    // Start
    i2c_master_start(cmd);
    // Master write follower address + write bit
    i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, I2C_MASTER_ACK);
    // Master write register address + send ack
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    // master stops
    i2c_master_stop(cmd);
    // This starts the I2C communication
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // master starts
    i2c_master_start(cmd1);
    // Master write follower address + read bit
    i2c_master_write_byte(cmd1, (addr << 1) | READ_BIT, I2C_MASTER_ACK);
    // Master reads in follower ack and data
    i2c_master_read_byte(cmd1, &data1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd1, &data2, I2C_MASTER_NACK);
    // Master nacks and stops
    i2c_master_stop(cmd1);
    // This starts the I2C communication
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd1, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd1);

    uint16_t two_byte_data = (data1 << 8 | data2);
    return two_byte_data;
}

static void configure_pin(void)
{
    gpio_reset_pin(Lidar1);
    gpio_reset_pin(Lidar2);
    gpio_reset_pin(Lidar3);
    gpio_set_direction(Lidar1, GPIO_MODE_OUTPUT);
    gpio_set_direction(Lidar2, GPIO_MODE_OUTPUT);
    gpio_set_direction(Lidar3, GPIO_MODE_OUTPUT);
}

static void change_addr(int num)
{

    uint16_t serial = readRegister(0x96, LIDARLite_ADDRESS);
    printf("Serial Number: %x\n", serial);

    int high = (serial >> 8) & 0xff;
    int low = serial & (0xff);

    writeRegister(0x18, high, LIDARLite_ADDRESS);
    writeRegister(0x19, low, LIDARLite_ADDRESS);
    if (num == 1)
    {
        writeRegister(0x1a, 0x60, LIDARLite_ADDRESS);
    }
    else
    {
        writeRegister(0x1a, 0x58, LIDARLite_ADDRESS);
    }

    writeRegister(0x1e, 0x08, LIDARLite_ADDRESS);
    printf("Changed Address!");
}

static void test()
{
    printf("\n>> Polling Lidar\n");
    while (1)
    {
        int addr;
        float *distance_m;
        prev_distance1 = distance1;
        for (int i = 0; i < 3; i++)
        {
            switch (i)
            {
            case 0:
                addr = Lidar1_addr;
                distance_m = &distance1;
                break;
            case 1:
                addr = Lidar2_addr;
                distance_m = &distance2;
                break;
            default:
                addr = Lidar3_addr;
                distance_m = &distance3;
            }
            // write to register 0x00 the value 0x04
            writeRegister(0x00, 0x04, addr);
            // READ REGISTER 0X01 UNTIL LSB GOES LOW
            // if LSB goes low then set flag to true
            int flag = 0;
            while (flag)
            {
                uint16_t data = readRegister(0x01, addr);
                // printf("DATA: %d; ", data);
                flag = data & (1 << 15);
                vTaskDelay(5);
            }

            uint16_t distance = readRegister(RegisterHighLowB, addr);
            *distance_m = (float)distance + 0.05;
            // printf("Lidar #%d: Distance %.2f m", i + 1, *distance_m);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        // printf("\n");
    }
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

////////////// SPEED SENSOR /////////////
////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT -100
#define WHEEL_SPEED_GPIO_A 34
#define WHEEL_SPEED_GPIO_B 2

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}
////////////// SPEED SENSOR /////////////
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
////////////////////////     Buggy Motion Control    ///////////////////////////////

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (S_SERVO_MAX_PULSEWIDTH_US - S_SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + S_SERVO_MIN_PULSEWIDTH_US;
}

static inline uint32_t ESC_SPEED(float speed)
{
    uint32_t ESC_value;

    if (speed >= 0)
    {
        ESC_value = ((ESC_MAX_PULSEWIDTH_US - ESC_NEUTRAL_PULSEWIDTH_US) * speed) + ESC_NEUTRAL_PULSEWIDTH_US;
    }
    else
    {
        ESC_value = ((ESC_NEUTRAL_PULSEWIDTH_US - ESC_MIN_PULSEWIDTH_US) * speed) + ESC_NEUTRAL_PULSEWIDTH_US;
    }

    ESP_LOGI(TAG_1, "ESC Value: %ld", ESC_value);

    return ESC_value;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

static void check_efuse(void)
{
    // Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

static inline uint32_t Forward_Collision_Senseor(void)
{

    uint32_t adc_reading = 0;
    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (unit == ADC_UNIT_1)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        else
        {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    // Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    // float distance = (voltage/1024)/5
    float distance = (voltage * 5) / 10;
    distance = distance - 18;
    // printf("Raw: %ld\tVoltage: %ldmV\tDistance: %.2f cm\n", adc_reading, voltage, distance);

    return distance;
}

#define KP 0.75
#define KI 0.5
#define KD 0.0

// Define limits for PID controller output
#define OUT_MIN 0.0
#define OUT_MAX 1023.0

// Define the sample time for the PID controller (in milliseconds)
#define SAMPLE_TIME_MS 100

// Define the timeout for the PID controller (in milliseconds)
#define TIMEOUT_MS 5000

// Define the target value for the system being controlled
#define TARGET_VALUE 400.00

QueueHandle_t input_queue, output_queue;

// // Define the PID controller task
// void PID(void *pvParameters)
// {
//     // Initialize variables
//     TickType_t last_time = xTaskGetTickCount();
//     float integral = 0.0;
//     float derivative = 0.0;
//     float last_error = 0.0;
//     float output = 0.0;
//     float input = 0.0;
//     float error = 0.0;
//     TickType_t timeout = xTaskGetTickCount() + pdMS_TO_TICKS(TIMEOUT_MS);

//     while (1) {
//         // Check if timeout has occurred
//         if (xTaskGetTickCount() >= timeout) {
//             ESP_LOGI("PID", "Timeout occurred, stopping PID controller task");
//             vTaskDelete(NULL);
//         }

//         // Read input from the input queue

//             // Calculate error
//             error = TARGET_VALUE - input;
//             printf("Error = %f\n", error);
//             // Calculate proportional term
//             float proportional = KP * error;

//             // Calculate integral term
//             integral += error * SAMPLE_TIME_MS / 1000.0;
//             float integral_term = KI * integral;

//             // Calculate derivative term
//             derivative = (error - last_error) / (SAMPLE_TIME_MS / 1000.0);
//             float derivative_term = KD * derivative;

//             // Calculate output and constrain within limits
//             output = proportional + integral_term + derivative_term;
//             if (output > OUT_MAX) {
//                 output = OUT_MAX;
//             } else if (output < OUT_MIN) {
//                 output = OUT_MIN;
//             }

//             // Write output to the output queue
//             xQueueSend(output_queue, &output, portMAX_DELAY);

//             // Save error for next iteration
//             last_error = error;

//             // Save time for next iteration
//             last_time = xTaskGetTickCount();

//         // Delay until next iteration
//         //vTaskDelayUntil(&last_time, pdMS_TO_TICKS(SAMPLE_TIME_MS));
//     }
// }

// static float PID() {

//     // error = setPoint - input
//     // intagral of error = integral += error * elapsedTime
//     // derivative of error =  derivative = (error - prev_error)/elapsedTime
//     // output = Kp * error + Ki * integral + Kd * derivative
//     // prev_error = error
//     // prev_time = current_time
//     while(1)
//     {
//         error = setPoint - (LID)
//     }

//     return e;
// }

float qr_code_time_start = 0;
float qr_code_time_end = 0;
float final_time = 0;
void app_main(void)
{

    // float integral, derivative, error, prev_error, output, dt;

    /////////////// Steering Servo ///////////////
    ///////////////////////////////////////////////////////////////////////////////////

    ESP_LOGI(TAG_1, "Create timer and operator");
    mcpwm_timer_handle_t timer1 = NULL;
    mcpwm_timer_config_t timer_config1 = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config1, &timer1));

    mcpwm_oper_handle_t oper1 = NULL;
    mcpwm_operator_config_t operator_config1 = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config1, &oper1));

    ESP_LOGI(TAG_1, "Connect timer and operator for steering servo");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, timer1));

    ESP_LOGI(TAG_1, "Create comparator and generator from the operator for steering servo");
    mcpwm_cmpr_handle_t comparator1 = NULL;
    mcpwm_comparator_config_t comparator_config1 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &comparator_config1, &comparator1));

    mcpwm_gen_handle_t generator1 = NULL;
    mcpwm_generator_config_t generator_config1 = {
        .gen_gpio_num = STEERING_SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &generator_config1, &generator1));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(0)));

    ESP_LOGI(TAG_1, "Set generator action on timer and compare event for steering servo");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator1, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG_1, "Enable and start timer for steering servo");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer1));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP));

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////// ESC Servo ///////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ESP_LOGI(TAG_1, "Create timer and operator");
    mcpwm_timer_handle_t timer2 = NULL;
    mcpwm_timer_config_t timer_config2 = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config2, &timer2));

    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t operator_config2 = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config2, &oper2));

    ESP_LOGI(TAG_1, "Connect timer and operator for ESC servo");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer2));

    ESP_LOGI(TAG_1, "Create comparator and generator from the operator for ESC servo");
    mcpwm_cmpr_handle_t comparator2 = NULL;
    mcpwm_comparator_config_t comparator_config2 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config2, &comparator2));

    mcpwm_gen_handle_t generator2 = NULL;
    mcpwm_generator_config_t generator_config2 = {
        .gen_gpio_num = ESC_SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config2, &generator2));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(0)));

    ESP_LOGI(TAG_1, "Set generator action on timer and compare event for ESC servo");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG_1, "Enable and start timer for ESC servo");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer2));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP));

    /////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////

    //////////// ULTRASONIC SENSOR /////////
    ///////////////////////////////////////////////////////////////////////////////////

    // Check if Two Point or Vref are burned into eFuse
    check_efuse();
    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    float FRONT_DISTANCE;
    //////////// ULTRASONIC SENSOR /////////
    ///////////////////////////////////////////////////////////////////////////////////

    ////////////// WHEEL SPEED SENSOR ///////
    ///////////////////////////////////////////////////////////////////////////////////
    ESP_LOGI(TAG_1, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    ESP_LOGI(TAG_1, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    ESP_LOGI(TAG_1, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = WHEEL_SPEED_GPIO_A,
        .level_gpio_num = WHEEL_SPEED_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = WHEEL_SPEED_GPIO_B,
        .level_gpio_num = WHEEL_SPEED_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_LOGI(TAG_1, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_LOGI(TAG_1, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++)
    {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));
    ESP_LOGI(TAG_1, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG_1, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG_1, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(WHEEL_SPEED_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    ////////////// WHEEL SPEED SENSOR ///////
    ///////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////    LIDAR SENSORS   ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    configure_pin();
    // Routine
    gpio_set_level(Lidar1, 1);
    gpio_set_level(Lidar2, 0);
    gpio_set_level(Lidar3, 0);
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(1000));
    i2c_scanner();
    change_addr(0);
    i2c_scanner();
    gpio_set_level(Lidar2, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    i2c_scanner();
    change_addr(1);
    i2c_scanner();
    gpio_set_level(Lidar3, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_scanner();

    // Create task to poll ADXL343
    //sxTaskCreate(test, "test", 4096, NULL, 5, NULL);
    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////

    int angle = 0;
    int step = 2;
    int neutral = 0;
    int flag = 1;
    float LIDAR_1_DIST, LIDAR_2_DIST, PID_input;

    vTaskDelay(300 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(angle)));

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(neutral)));
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(0.14)));

    // initial steering set to be straight
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-5)));
    // vTaskDelay(10);
    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(neutral)));
    //  input_queue = xQueueCreate(1, sizeof(float));
    //  output_queue = xQueueCreate(1, sizeof(float));
    mux = xSemaphoreCreateMutex();

    // Timer queue initialize -- 2023: modified
    timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!timer_queue)
    {
        ESP_LOGE(TAG_TIMER, "Creating queue failed");
        return;
    }

    // Create task to handle timer-based events -- no changes
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    //*******************************************************//

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    //*******************************************************//

    xTaskCreate(udp_server_task, "udp_server_task", 4096, NULL, 5, NULL);

    int qflag = 0;
    int UDP_send = 0;
    while (1)
    {

        printf("QR code is %c", qr_code);

        if (qr_code == 'g' && qflag == 0)
        {
            qflag = 1;
            printf("GOOOOOOOOOOOOOOOO\n\n\n");
            qr_code_time_start = esp_timer_get_time();
            // UDP_send = 1;
        }
        if (qr_code == '1' && qflag == 1)
        {
            qflag = 2;
            qr_code_time_end = esp_timer_get_time();
            final_time = qr_code_time_end - qr_code_time_start;
            final_time = final_time / 1000000;

            printf("Time %f\n\n", final_time);
            qr_code_time_start = esp_timer_get_time();

            UDP_send = 1;
        }

        if (qr_code == '2' && qflag == 2)
        {
            qflag = 3;
            qr_code_time_end = esp_timer_get_time();
            final_time = qr_code_time_end - qr_code_time_start;
            final_time = final_time / 1000000;

            printf("Time %f\n\n", final_time);
            qr_code_time_start = esp_timer_get_time();

            UDP_send = 1;
        }
        if (qr_code == '3' && qflag == 3)
        {
            qflag = 4;
            qr_code_time_end = esp_timer_get_time();
            final_time = qr_code_time_end - qr_code_time_start;
            final_time = final_time / 1000000;
            printf("Time %f\n\n", final_time);
            qr_code_time_start = esp_timer_get_time();

            UDP_send = 1;
        }
        if (qr_code == '4' && qflag == 4)
        {
            qflag = 0;
            qr_code_time_end = esp_timer_get_time();
            final_time = qr_code_time_end - qr_code_time_start;
            final_time = final_time / 1000000;
            printf("Time %f", final_time);

            UDP_send = 1;
        }
        UDP_send = 1;

        char final_time_char[20];
        sprintf(final_time_char, "%f", final_time);
        printf("final time char is %s", final_time_char);

        sprintf(PAYLOAD, "%f", final_time);

        uint16_t displaybuffer[8];
        displaybuffer[0] = alphafonttable[final_time_char[0] - 32];
        displaybuffer[1] = alphafonttable[final_time_char[1] - 32];
        displaybuffer[2] = alphafonttable[final_time_char[2] - 32];
        displaybuffer[3] = alphafonttable[final_time_char[3] - 32];

        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i = 0; i < 8; i++)
        {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd4);

        // for (int i = 0; i < 8; i++) {
        //     printf("%04x\n", displaybuffer[i]);
        // }

        if (ret == ESP_OK)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            printf("Wrote to Display\n");
        }

        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR); // Destination IP
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT_SND); // Destination Port
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");
        // while (UDP_send == 1)
        // {
        int err = sendto(sock, PAYLOAD, strlen(PAYLOAD), 0,
                         (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }
        else
        {
            ESP_LOGI(TAG, "Message Sent");
        }
        UDP_send = 1;
        // }
        close(sock);
        vTaskDelay(10 / portTICK_PERIOD_MS);

        //////////////////////////////////////////////////////////////////////////////

        // getting front distance every time we loop through
        FRONT_DISTANCE = Forward_Collision_Senseor();

        LIDAR_1_DIST = distance1;
        LIDAR_2_DIST = distance3;
        PID_input = (LIDAR_1_DIST - LIDAR_2_DIST);

        // float input = PID_input;
        // printf("INPUT = %f\n", input);
        // xQueueSend(input_queue, &input, portMAX_DELAY);

        printf("Lidar1 distance = %f\n", LIDAR_1_DIST);
        printf("Lidar2 distance = %f\n", LIDAR_2_DIST);

        //  // Start the PID controller task
        // xTaskCreate(PID, "PID", 4096, NULL, 5, NULL);

        /////////////////////////       Car Logic       ///////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        // STATE 0 = WALL IN FRONT && WALL ON RIGHT
        // STATE 1 = NO WALL IN FRONT && WALL ON RIGHT

        // IF FRONT_DISTANCE > 80 && LIDAR1/2 < 65 --> STATE 0

        // IF FRONT_DISTANCE <= 80 --> WALL IN FRONT --> STATE = 1
        bool state;
        bool stop_state = 0;  // front wall
        bool drive_state = 0; // no front wall

        if (FRONT_DISTANCE < 50)
        {
            stop_state = 1;
            drive_state = 0;
            ESP_LOGI(TAG_1, "DANGER Ahead %fcm away", FRONT_DISTANCE);
        }
        else if (FRONT_DISTANCE > 50)
        {
            stop_state = 0;
            drive_state = 1;
            ESP_LOGI(TAG_1, "Object Ahead %fcm away", FRONT_DISTANCE);
        }
        else
        {
            stop_state = 0;
            drive_state = 0;
            ESP_LOGI(TAG_1, "Oh Oh...Im stuck ( Object %fcm away )", FRONT_DISTANCE);
        }

        if (drive_state == 1 && stop_state == 0)
        {
            if (wasdfcontrols == 'w')
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(0.15)));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-5)));

            }
            if (wasdfcontrols == 'a')
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(0.15)));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(20)));
            }
            if (wasdfcontrols == 'd')
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(0.15)));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-25)));
            }
            if (wasdfcontrols == 's')
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(-0.15)));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-5)));
            }
        }

        else if (drive_state == 0 && stop_state == 1)
        {
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(-0.14)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-5)));
            //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(neutral)));

            if (wasdfcontrols == 's'){
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(-0.22)));
            }

            if (wasdfcontrols == 'w')
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(neutral)));
            }
        }

        else
        {
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, example_angle_to_compare(-5)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, ESC_SPEED(neutral)));
        }

        ///////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        // Wheel speed sensor task
        //  Report counter value
        int pulse_count = 0;
        int event_count = 0;
    }
}
