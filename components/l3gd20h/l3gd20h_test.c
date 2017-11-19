#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "esp8266_wrapper.h"

#include "l3gd20h.c"

// #define SPI_USED

#define I2C_BUS  0

static l3gd20h_sensor_t* sensor = 0;

void user_task(void *pvParameters)
{
    l3gd20h_set_mode (sensor, l3gd20h_normal_odr_100, 3, true, true, true);

    /*    
    l3gd20h_set_fifo_mode (sensor, l3gd20h_fifo);

    l3gd20h_int1_config_t int1_config;
    l3gd20h_int1_source_t int1_source;
    
    l3gd20h_get_int1_config (sensor, &int1_config);
    
    int1_config.z_low_enabled = false;
    int1_config.z_high_enabled = true;
    int1_config.z_threshold = 1000;
    
    int1_config.filter = l3gd20h_only_hpf;
    int1_config.and_combination = false;
    int1_config.duration = 15;
    int1_config.wait_enabled = true;
    
    l3gd20h_set_int1_config (sensor, &int1_config);
    */
    
    int count = 0;

    while (1)
    {
        l3gd20h_float_data_t  data;

        if (l3gd20h_new_data (sensor) &&
            l3gd20h_get_float_data (sensor, &data))
            printf("%.3f L3GD20H Sensor: x=%f y=%f z=%f\n",
                   (double)sdk_system_get_time()*1e-3, data.x, data.y, data.z);

	/*        
        l3gd20h_raw_data_fifo_t fifo;
        uint8_t level = l3gd20h_new_data (sensor);
        printf("%.3f L3GD20H Sensor: level=%d\n", (double)sdk_system_get_time()*1e-3, level);
        if (level)
        {
            uint8_t len = l3gd20h_get_raw_data_fifo (sensor, fifo);
            printf("%.3f L3GD20H Sensor: len=%d\n", (double)sdk_system_get_time()*1e-3, level);
            if (len)
            {
                for (int i=0; i < len; i++)
                    printf("%.3f L3GD20H Sensor: x=%f (%d) y=%f (%d) z=%f (%d)\n",
                       (double)sdk_system_get_time()*1e-3, 
                       0.0, fifo[i].x, 0.0, fifo[i].y, 0.0, fifo[i].z);

            }
        }
	*/
        
              
        vTaskDelay(20);
        // vTaskDelay(count % 2 ? 1 : 20);
        
        count++;
    }
}

int main (int argc, char* argv[])
{
    char devname[BUFSIZ];

    int bus  = 0;
    int addr = 0;
    int cs   = 0;
    
    #ifdef SPI_USED
    if (argc < 3)
    {
        printf ("Usage: sudo l3gd20h_test <bus> <cs> \n\n\t /dev/spidev<bus>.<cs>\n\n");
        exit (1);
    }
    
    bus = atoi (argv[1]);
    cs  = atoi (argv[2]);
    
    if (bus >= SPI_MAX_BUS || cs >= SPI_MAX_CS)
    {
        printf ("Wrong value: bus=%d, cs=%d\n", bus, cs);
        exit (1);
    }
    
    sprintf(devname, "/dev/spidev%d.%d", bus, cs);
    
    if ((spi_device[bus][cs] = open(devname, O_RDWR)) < 0)
    {
        printf ("Could not open SPI device %s\n", devname);
        exit(1);
    }

    #else  // I2C used

    if (argc < 2)
    {
        printf ("Usage: sudo l3gd20h_test <if> \n\n\t /dev/i2c-<if>\n\n");
        exit (1);
    }

    bus  = I2C_BUS;
    addr = L3GD20H_I2C_ADDRESS_2;
    
    sprintf(devname, "/dev/i2c-%s", argv[1]);

    if ((i2c_bus[bus] = open(devname, O_RDWR)) < 0)
    {
        printf ("Could not open I2C device %s\n", devname);
        exit(1);
    }

    // i2c_read_block_data_possible = false;

    #endif

    if ((sensor = l3gd20h_init_sensor (bus, addr, cs)))
    {
        user_task (0);
    }
}
