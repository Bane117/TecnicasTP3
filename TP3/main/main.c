#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "driver/spi_master.h" //Driver de la libreria
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define MOSI_GPIO 23
#define MISO_GPIO 19
#define SPI_CLK 18
#define SPI_CS 5



spi_device_handle_t spi_device_handle;


static esp_err_t init_spi (void){

	spi_bus_config_t spi_bus_config ={};

	spi_bus_config.mosi_io_num = MOSI_GPIO;
	spi_bus_config.miso_io_num = MISO_GPIO;
	spi_bus_config.sclk_io_num = SPI_CLK;
	spi_bus_config.quadwp_io_num = -1;
	spi_bus_config.quadhd_io_num = -1;
	spi_bus_config.max_transfer_sz = 32;

	spi_device_interface_config_t spi_device_interface_config = {};

	spi_device_interface_config.mode = 0;
	spi_device_interface_config.duty_cycle_pos = 128;
	spi_device_interface_config.clock_speed_hz = 1000000;
	spi_device_interface_config.spics_io_num = SPI_CS;
	spi_device_interface_config.queue_size = 1;
	spi_device_interface_config.flags = SPI_DEVICE_HALFDUPLEX;
	spi_device_interface_config.pre_cb = NULL;
	spi_device_interface_config.post_cb = NULL;

	spi_bus_initialize(2, &spi_bus_config, SPI_DMA_CH_AUTO);
	spi_bus_add_device(2, &spi_device_interface_config, &spi_device_handle);

	return ESP_OK;
}

	//0x12 , 100101010

static void spi_write(uint8_t reg, uint8_t  value){
		uint8_t data[2]= {reg, value};
		spi_transaction_t spi_transaction = {
				.tx_buffer = data,
				.length = 16
		};

		spi_device_transmit (spi_device_handle, &spi_transaction);

}



static void spi_read(uint8_t reg, uint8_t *value, int len) {
    uint8_t tx_data = reg | 0x80; // El bit 7 debe ser 1 para lectura
    spi_transaction_t spi_transaction = {
        .length = 8 + (8 * len), // 8 bits del registro + 8 bits por cada valor leído
        .tx_buffer = &tx_data,
        .rx_buffer = value
    };

    spi_device_transmit(spi_device_handle, &spi_transaction);
}



static esp_err_t bmp280_init (void) {
    // Resetear el BMP280
    spi_write(0xE0, 0xB6);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Leer el chip ID para verificar la comunicación
    uint8_t chip_id = 0;
    spi_read(0xD0, &chip_id, 1);
    if (chip_id != 0x58) {
        ESP_LOGE("BMP280", "Fallo en la comunicación con el sensor BMP280");
        return ESP_FAIL;
    }

    // Configurar el BMP280
    spi_write(0xF4, 0x27); // ctrl_meas: modo normal, oversampling de presión y temperatura en x1
    spi_write(0xF5, 0xA0); // config: tiempo de espera 1000ms, filtro en x16

    return ESP_OK;
}



static void read_bmp280_data(int32_t *temperature, int32_t *pressure) {
    uint8_t data[6];

    // Leer los registros de datos de presión y temperatura
    spi_read(0xF7, data, 6);

    // Combinar los bytes leídos en valores de presión y temperatura
    int32_t adc_P = (int32_t)(((uint32_t)(data[0] << 16) | (uint32_t)(data[1] << 8) | (uint32_t)data[2]) >> 4);
    int32_t adc_T = (int32_t)(((uint32_t)(data[3] << 16) | (uint32_t)(data[4] << 8) | (uint32_t)data[5]) >> 4);

    // Calibrar y convertir los valores leídos (aquí necesitarás los coeficientes de calibración, no incluidos en este ejemplo)
    *temperature = adc_T; // Estos valores deben ser calibrados
    *pressure = adc_P;    // Estos valores deben ser calibrados
}




void app_main(void) {
    ESP_ERROR_CHECK(init_spi());
    ESP_ERROR_CHECK(bmp280_init());

    int32_t temperature, pressure;

    while (true) {
        read_bmp280_data(&temperature, &pressure);

        // Aquí puedes añadir la conversión de los valores brutos a unidades reales utilizando los coeficientes de calibración




        // Imprimir los valores leídos (en unidades brutas por ahora)
        printf("Temperature: %ld\n", temperature);
        printf("Pressure: %ld\n", pressure);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
