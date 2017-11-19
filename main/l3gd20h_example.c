/**
 * Simple example with one sensor connected to I2C or SPI. It demonstrates the
 * different approaches to fetch the data. Either one of the interrupt signals
 * for axes movement wake up *INT1* and data ready interrupt *INT2* is used
 * or the new data are fetched periodically.
 *
 * Harware configuration:
 *
 *   I2C   +-------------------------+     +----------+
 *         | ESP8266  Bus 0          |     | L3GD20H  |
 *         |          GPIO 5 (SCL)   ------> SCL      |
 *         |          GPIO 4 (SDA)   ------- SDA      |
 *         |          GPIO 13        <------ INT1     |
 *         |          GPIO 12        <------ INT2/DRDY|
 *         +-------------------------+     +----------+
 *
 *         +-------------------------+     +----------+
 *         | ESP32    Bus 0          |     | L3GD20H  |
 *         |          GPIO 16 (SCL)  >-----> SCL      |
 *         |          GPIO 17 (SDA)  ------- SDA      |
 *         |          GPIO 22        <------ INT1     |
 *         |          GPIO 23        <------ INT2/DRDY|
 *         +-------------------------+     +----------+
 *
 *   SPI   +-------------------------+     +----------+
 *         | ESP8266  Bus 1          |     | L3GD20H  |
 *         |          GPIO 14 (SCK)  ------> SCK      |
 *         |          GPIO 13 (MOSI) ------> SDI      |
 *         |          GPIO 12 (MISO) <------ SDO      |
 *         |          GPIO 2  (CS)   ------> CS       |
 *         |          GPIO 5         <------ INT1     |
 *         |          GPIO 4         <------ INT2/DRDY|
 *         +-------------------------+     +----------+

 *         +-------------------------+     +----------+
 *         | ESP32    Bus 0          |     | L3GD20H  |
 *         |          GPIO 16 (SCK)  ------> SCK      |
 *         |          GPIO 17 (MOSI) ------> SDI      |
 *         |          GPIO 18 (MISO) <------ SDO      |
 *         |          GPIO 19 (CS)   ------> CS       |
 *         |          GPIO 22        <------ INT1     |
 *         |          GPIO 23        <------ INT2/DRDY|
 *         +-------------------------+     +----------+
 */

// use following constants to define the example mode
// #define INT_USED
// #define SPI_USED

#include <string.h>

/* -- platform dependent includes ----------------------------- */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp8266_wrapper.h"

#include "l3gd20h.h"

#else  // ESP8266 (esp-open-rtos)

#define TASK_STACK_DEPTH 256

#include <stdio.h>

#include "espressif/esp_common.h"
#include "espressif/sdk_private.h"

#include "esp/uart.h"
#include "i2c/i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "l3gd20h/l3gd20h.h"

#endif  // ESP_PLATFORM

/** -- platform dependent definitions ------------------------------ */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth
#define TASK_STACK_DEPTH 2048

// define SPI interface for L3GD20H sensors
#define SPI_BUS       HSPI_HOST
#define SPI_SCK_GPIO  16
#define SPI_MOSI_GPIO 17
#define SPI_MISO_GPIO 18
#define SPI_CS_GPIO   19

// define I2C interfaces for L3GD20H sensors
#define I2C_BUS       0
#define I2C_SCL_PIN   16
#define I2C_SDA_PIN   17
#define I2C_FREQ      400000

// define GPIOs for interrupt
#define INT1_PIN      22
#define INT2_PIN      23

#else  // ESP8266 (esp-open-rtos)

// user task stack depth
#define TASK_STACK_DEPTH 256

// define SPI interface for L3GD20H sensors
#define SPI_BUS       1
#define SPI_CS_GPIO   2   // GPIO 15, the default CS of SPI bus 1, can't be used

// define I2C interfaces for L3GD20H sensors
#define I2C_BUS       0
#define I2C_SCL_PIN   5
#define I2C_SDA_PIN   4
#define I2C_FREQ      I2C_FREQ_100K

// define GPIOs for interrupt
#ifdef SPI_USED
#define INT1_PIN      5
#define INT2_PIN      4
#else
#define INT1_PIN      13
#define INT2_PIN      12
#endif  // SPI_USED

#endif  // ESP_PLATFORM

/* -- user tasks ---------------------------------------------- */

static l3gd20h_sensor_t* sensor;

#ifdef INT_USED
/**
 * In this case, axes movement wake up interrupt *INT1*  and data ready
 * interrupt *INT2* are used. While data ready interrupt *INT2* is generated
 * every time new data are available or the FIFO status changes, the axes
 * movement wake up interrupt *INT1* is triggered when output data across
 * defined thresholds.
 *
 * When interrupts are used, the user has to define interrupt handlers that
 * fetches the data directly or triggers a task, that is waiting to fetch the
 * data. In this example, a task is defined which suspends itself
 * in each cycle to wait for fetching the data. The task is resumed by the
 * the interrupt handler.
 */

static QueueHandle_t gpio_evt_queue = NULL;

// User task that fetches the sensor values.

void user_task_interrupt (void *pvParameters)
{
    uint32_t gpio_num;

    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            if (gpio_num == INT1_PIN)
            {
                l3gd20h_int1_source_t source;
                l3gd20h_float_data_t  data;

                // get interrupt source 
                l3gd20h_get_int1_source (sensor, &source);

                // if data ready interrupt, get the results and do something with them
                if (source.active &&
                    l3gd20h_get_float_data (sensor, &data))
                    printf("%.3f L3GD20H Sensor INT1: x=%f y=%f z=%f\n",
                           (double)sdk_system_get_time()*1e-3, 
                           data.x, data.y, data.z);
            }
            else if (gpio_num == INT2_PIN)
            {
                l3gd20h_int2_source_t source;
                l3gd20h_float_data_t  data;

                // get interrupt source 
                l3gd20h_get_int2_source (sensor, &source);

                // if data ready interrupt, get the results and do something with them
                if (source.data_ready &&
                    l3gd20h_new_data (sensor) &&
                    l3gd20h_get_float_data (sensor, &data))
                    printf("%.3f L3GD20H Sensor INT2: x=%f y=%f z=%f\n",
                           (double)sdk_system_get_time()*1e-3, 
                           data.x, data.y, data.z);
            }
        }
    }
}

// Interrupt handler which resumes user_task_interrupt on interrupt

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)
static void IRAM_ATTR int_signal_handler(void* arg)
{
    uint32_t gpio = (uint32_t) arg;
#else  // ESP8266 (esp-open-rtos)
void int_signal_handler (uint8_t gpio)
{
#endif
    // send an event with GPIO to the interrupt user task
    xQueueSendFromISR(gpio_evt_queue, &gpio, NULL);
}

#else

/*
 * In this example, user task fetches the sensor values every seconds.
 */

void user_task_periodic(void *pvParameters)
{
    vTaskDelay (10);
    
    while (1)
    {
        l3gd20h_float_data_t  data;

        // get the results and do something with them
        if (l3gd20h_new_data (sensor) &&
            l3gd20h_get_float_data (sensor, &data))
            printf("%.3f L3GD20H Sensor: x=%f y=%f z=%f\n",
                   (double)sdk_system_get_time()*1e-3, data.x, data.y, data.z);

        // passive waiting until 1 second is over
        vTaskDelay(10);
    }
}

#endif

/* -- main program ---------------------------------------------- */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)
void app_main()
#else  // ESP8266 (esp-open-rtos)
void user_init(void)
#endif
{
    #ifdef ESP_OPEN_RTOS  // ESP8266
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    #endif

    vTaskDelay(1);

    /** -- MANDATORY PART -- */

    #ifdef SPI_USED

    // init the sensor connnected to SPI
    #ifdef ESP_OPEN_RTOS
    sensor = l3gd20h_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    #else
    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num=SPI_MISO_GPIO,
        .mosi_io_num=SPI_MOSI_GPIO,
        .sclk_io_num=SPI_SCK_GPIO,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    if (spi_bus_initialize(SPI_BUS, &spi_bus_cfg, 1) == ESP_OK)
        sensor = l3gd20h_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    #endif
    
    #else

    // init all I2C bus interfaces at which L3GD20H sensors are connected
    i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    
    // init the sensor with slave address L3GD20H_I2C_ADDRESS_2 connected I2C_BUS.
    sensor = l3gd20h_init_sensor (I2C_BUS, L3GD20H_I2C_ADDRESS_2, 0);

    #endif
    
    if (sensor)
    {
        // start periodic measurement for all three axes
        l3gd20h_set_mode (sensor, l3gd20h_normal_odr_12_5, 0, true, true, true);

        #if !defined(INT_USED)

        // create a user task that fetches data from sensor periodically
        xTaskCreate(user_task_periodic, "user_task_periodic", TASK_STACK_DEPTH, NULL, 2, NULL);

        #else // INT_USED

        // create a task that is triggered only in case of interrupts to fetch the data
        xTaskCreate(user_task_interrupt, "user_task_interrupt", TASK_STACK_DEPTH, NULL, 2, NULL);

        // create event queue
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

        #ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)
        
        // configure interupt pins for *INT1* and *INT2* signals
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = ((uint64_t)(((uint64_t)1)<< INT1_PIN) | (((uint64_t)1)<< INT2_PIN)),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = true,
            .intr_type = GPIO_INTR_POSEDGE
        };
        gpio_config(&gpio_cfg);

        // set interrupt handler
        gpio_install_isr_service(0);
        gpio_isr_handler_add(INT1_PIN, int_signal_handler, (void*)INT1_PIN);
        gpio_isr_handler_add(INT2_PIN, int_signal_handler, (void*)INT2_PIN);

        #else  // ESP8266 (esp-open-rtos)

        // configure interupt pins for *INT1* and *INT2* signals and set the interrupt handler
        gpio_set_interrupt(INT1_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);
        gpio_set_interrupt(INT2_PIN, GPIO_INTTYPE_EDGE_POS, int_signal_handler);

        #endif

        // enable data ready interrupt signal *INT2*
        l3gd20h_enable_int2 (sensor, l3gd20h_data_ready, true);

        /*
        l3gd20h_config_hpf (sensor, l3gd20h_normal, 0, 0);
        l3gd20h_select_output_filter (sensor, l3gd20h_only_hpf);
        
        l3gd20h_int1_config_t int1_config;

        l3gd20h_get_int1_config (sensor, &int1_config);
        
        int1_config.x_high_enabled = true;
        int1_config.x_threshold = 13374;  // 100 dps at 245 dps scale
        int1_config.y_high_enabled = true;
        int1_config.y_threshold = 13374;  // 100 dps at 245 dps scale
        int1_config.z_high_enabled = true;
        int1_config.z_threshold = 13374;  // 100 dps at 245 dps scale
        int1_config.latch_interrupt = true;
        int1_config.and_combination = false;
        int1_config.filter = l3gd20h_only_hpf;
        
        l3gd20h_set_int1_config (sensor, &int1_config);
        */
        #endif

        vTaskDelay (20);
        
    }
}

