#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "driver/spi_master.h" //Driver de la libreria
#include "esp_log.h"

#define MOSI_GPIO 23
#define MISO_GPIO 19
#define SPI_CLK 18
#define SPI 5


spi_device_handle_t spi_device_handle;


static esp_esp_err_t init_spi (void){

	spi_bus_config_t spi_bus_config ={};

	spi_bus_config.mosi_io_num = MOSI_GPIO;
	spi_bus_config.miso_io_num = MISO_GPIO;
	spi_bus_config.sclk_io_num = SPI_CLK;
	spi_bus_config.quedwp_io_num = -1;
	spi_bus_config.quadhd_io_num = -1;
	spi_bus_config.max_tranfer_sz = 32;

	spi_device_interface_config_t spi_device_interface_config = {};

	spi_device_interface_config.mode = 0;
	spi_device_interface_config.duty_cycle_pos = 128;
	spi_device_interface_config.clocl_speed_hz = 1000000;
	spi_device_interface_config.spics_io_num = SPI_CS;
	spi_device_interface_config.queue_size = 1;
	spi_device_interface_config.pre_cb = NULL;
	spi_device_interface_config.post_cb = NULL;

	spi_bus_initialize(SPI3_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
	spi_add_device(SPI3_HOST, &spi_device_interface_config, &spi_device_handle);

	return ESP_OK;
}


void app_main(void)
{
    while (true) {
        printf("Hello from app_main!\n");
        sleep(1);
    }
}
